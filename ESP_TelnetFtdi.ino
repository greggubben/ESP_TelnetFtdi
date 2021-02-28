// Convert Telnet connection to a TTL Serial connection

#include <WiFiManager.h>        // For managing the Wifi Connection
#include <ESP8266mDNS.h>        // For running OTA and Web Server
#include <WiFiUdp.h>            // For running OTA
#include <ArduinoOTA.h>         // For running OTA
//#include <TelnetSerial.h>       // For debugging via Telnet
#include <SoftwareSerial.h>     // For connecting to serial device
#include <algorithm>        // std::min

/*
 * This code will handle both the VCF Badge and TIM (DATAC 1000).
 * It accepts a Telnet session and connects to the 6502 over TTL.
 * One 1 can be uncommmented at a time.
 */
#define TARGET_VCFBADGE     // Uncomment when connecting to the VCF Badge
//#define TARGET_TIM          // Uncomment when connecting to TIM (DATAC 1000)

#if !defined(TARGET_VCFBADGE) && !defined(TARGET_TIM)
#error Must define one TARGET_VCFBADGE or TARGET_TIM
#endif
#if defined(TARGET_VCFBADGE) && defined(TARGET_TIM)
#error Can only define one TARGET_VCFBADGE or TARGET_TIM
#endif

#ifdef TARGET_TIM
// Device Info
const char* devicename = "TIM";
const char* devicepassword = "timAdmin";
unsigned long baud = 300;
#endif

#ifdef TARGET_VCFBADGE
// Device Info
const char* devicename = "VCFBadge";
const char* devicepassword = "vcfAdmin";
unsigned long baud = 9600;
#endif

// Software Serial variables
#define TELNETCLIENT_STACK_PROTECTOR  512 // bytes
#define SERIAL_BUFFER_SIZE_RX  1024 // bytes
const unsigned int rxPin = D2;
const unsigned int txPin = D1;
SoftwareSerial deviceSerial(rxPin, txPin);

//for using LED as a startup status indicator
#include <Ticker.h>
Ticker ticker;
boolean ledState = LOW;   // Used for blinking LEDs when WifiManager in Connecting and Configuring

// On board LED used to show status
#ifndef LED_BUILTIN
#define LED_BUILTIN 13 // ESP32 DOES NOT DEFINE LED_BUILTIN
#endif
const int ledPin =  LED_BUILTIN;  // the number of the LED pin

// Telnet Serial variables
//TelnetSerial telnetSerial;  // Manage Telnet connection to receive Serial data
//Stream *usbSerial;          // Pointer to USB/Hardware Serial for fallback debugging

// Telnet Server variables
unsigned long port = 23;
WiFiServer telnetServer = WiFiServer(port);
WiFiClient telnetClient;

// Used for running a simple program during loop to blink led
// replace with real variables
unsigned long previousMillis = 0;     // will store last time LED was updated
const long interval = 2000;           // interval at which to blink (milliseconds)


/*************************************************
 * Setup
 *************************************************/
void setup() {
  Serial.begin(115200);

  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // start ticker to slow blink LED strip during Setup
  ticker.attach(0.6, tick);


  //
  // Set up the Wifi Connection
  //
  WiFi.hostname(devicename);
  WiFi.mode(WIFI_STA);      // explicitly set mode, esp defaults to STA+AP
  
  WiFiManager wm;
  // wm.resetSettings();    // reset settings - for testing
  wm.setAPCallback(configModeCallback); //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  //if it does not connect it starts an access point with the specified name here  "AutoConnectAP"
  if (!wm.autoConnect(devicename,devicepassword)) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
  }
  Serial.println("connected");


  //
  // Set up the Multicast DNS
  //
  MDNS.begin(devicename);


  //
  // Set up OTA
  //
  // ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(devicename);
  ArduinoOTA.setPassword(devicepassword);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    //Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    //Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      //Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      //Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      //Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      //Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      //Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  // Setup Telnet Serial
  //telnetSerial.begin(115200);
  //usbSerial = telnetSerial.getOriginalSerial();

  // Let USB/Hardware Serial know where to connect.
  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.printf(" %d' to connect\n", port);


  //
  // Done with Setup
  //
  ticker.detach();          // Stop blinking the LED

  // Start the Telnet Server
  telnetServer.begin(port);
  telnetServer.setNoDelay(true);
  

  // Now start the Serial line
  // We don't want the startup junk to interfer with talking to the device
  deviceSerial.begin(baud);
  //deviceSerial.setRxBufferSize(SERIAL_BUFFER_SIZE_RX);
  deviceSerial.print('\n');

}


/*************************************************
 * Loop
 *************************************************/
void loop() {
  // Handle any requests
  ArduinoOTA.handle();
  MDNS.update();
  //telnetSerial.handle();

  // new clients?
  if (telnetServer.hasClient()) {
    if (!telnetClient.connected()) {
      // Accept new client
      telnetClient = telnetServer.available();
      Serial.println("New Client");
      // Dump incoming telnet characters
      //Serial.println("incoming data from telnet client");
      while (telnetClient.available()) {
        uint8_t b = telnetClient.read();
        //Serial.printf("0x%02X ",b);
        //Serial.println(b);
      }
      deviceSerial.println("V");
      //Serial.println("done");
    }
    else {
      // Already have a telnet client - reject
      telnetServer.available().println("busy");
      Serial.println("Server busy with active connection");
    }
  }

  // Send data from telnetClient to Serial Line
  while (telnetClient.available()) {
    deviceSerial.write(telnetClient.read());
  }

  // Determine max amount of data the Telnet Client can receive at a time
  size_t maxBytesToSendTelnetClient = 0;
  if (telnetClient) {
    // client.availableForWrite() returns 0 when !client.connected()
    size_t telnetClientWriteBufferSize = telnetClient.availableForWrite();
    if (telnetClientWriteBufferSize) {
      maxBytesToSendTelnetClient = telnetClientWriteBufferSize;
    }
    else {
      // warn but ignore congested clients
      Serial.println("Client is congested.");
    }
  }

  // Send data from Serial line to telnetClient
  size_t dataToSendLen = std::min((size_t)deviceSerial.available(), maxBytesToSendTelnetClient);
  dataToSendLen = std::min(dataToSendLen, (size_t)TELNETCLIENT_STACK_PROTECTOR);

  if (dataToSendLen) {
    uint8_t dataToSend[dataToSendLen];
    size_t dataReadFromSerialLen = deviceSerial.readBytes(dataToSend, dataToSendLen);
    // push UART data to connected telnet client
    // if client.availableForWrite() was 0 (congested)
    // and increased since then,
    // ensure write space is sufficient:
    if (telnetClient.availableForWrite() >= dataReadFromSerialLen) {
      size_t dataSentToTelnetClientLen = telnetClient.write(dataToSend, dataReadFromSerialLen);
      if (dataSentToTelnetClientLen != dataToSendLen) {
        Serial.printf("data length mismatch: to send:%zd readFromSerial:%zd sentToTelnetClient:%zd\n", dataToSendLen, dataReadFromSerialLen, dataSentToTelnetClientLen);
      }
    }
  }
/*
    while (deviceSerial.available()) {
      telnetClient.write(deviceSerial.read());
    }
*/  
}


/*************************************************
 * Callback Utilities during setup
 *************************************************/
 
/*
 * Blink the LED Strip.
 * If on  then turn off
 * If off then turn on
 */
void tick()
{
  //toggle state
  digitalWrite(ledPin, !digitalRead(ledPin));     // set pin to the opposite state
}

/*
 * gets called when WiFiManager enters configuration mode
 */
void configModeCallback (WiFiManager *myWiFiManager) {
  //Serial.println("Entered config mode");
  //Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  //Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}
