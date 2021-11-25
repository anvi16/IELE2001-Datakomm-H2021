


// Include libraries
#include <Arduino.h>                            // General Arduino C++ lib
#include <TinyGPS++.h>                          // GPS lib for easy extraction of GPS data
#include <SoftwareSerial.h>                     // Software serial lib (for serial com on any pin)
#include <TimeLib.h>                            // Time lib Arduino
#include <cstdio>
#include <iostream>                            
#include <TFT_eSPI.h>                           // TFT lib for control of OLED screen
#include <SPI.h>                                // SPI lib. TFT lib depends on this
#include <Wire.h>

#define DEBUG

// Include files with dependencies
#include "PurpaceMadeLib/sensors_ext/sensors_ext.h"         // File containging classes related to peripheral sensors (GPS, temp, hum, press)
#include "PurpaceMadeLib/ClusterCom/ClusterCom.h"           // ClusterCom library for peer to peer communicatision
#include "PurpaceMAdeLib/DisplayTTGO/DisplayTTGO.h"
#include "config.h"                                         // Config file for HW pins, baud etc.




// TFT class for OLED display control
TFT_eSPI tft = TFT_eSPI(135, 240);

// GPS class - Constructor takes GPS module baud rate and pins for PS, Tx and Rx
GPS gps(GPSBaud, serialBaud, psGPS, txGPS, rxGPS);

// Weather station class (for simple reading of weather station data)
WeatherStation ws(i2cBME280Adr);

// Ubidots object
Ubidots ubidots(UBIDOTS_TOKEN);

// ClusterCom object
ClusterCom CCom(rxRF, txRF, psRFRX, psRFTX, serialBaud);

// TTGO display
DisplayTTGO display;
/****************************************
 * Variabels Functions
 ****************************************/
String callbackPayload = "";
bool firstScan = HIGH;

// ClusterCom
uint8_t id = 0;     // Device id / Default = 0
String msg;         // Buffer for incoming message
uint8_t mt;         // Buffer for messageType



/****************************************
 * Auxiliar Functions
 ****************************************/

void UbisoftCallback(char *topic, byte *payload, unsigned int length)
{
  callbackPayload = "";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    callbackPayload.concat((char)payload[i]);
  }
  // Serial.println(callbackPayload);

  // for (int i = 0; i < length; i++)
  // {
  //   Serial.print((char)payload[i]);
  // }
  Serial.println();
}


// Functionaly the same as the "delay()" function.
// However, instead of just waiing, the unit is put into light sleep to save power
void espDelay(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}


void auxLoop()
{
  gps.refresh(HIGH);                                // Update data from GPS

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_GREEN);
  tft.print("Test");
}


void commLoop()
{
/*
  WeatherData weatherData; 

  for (int unit = 0; unit < 100; unit++){
    weatherData.lat = 0;
    weatherData.lng = 0;
    weatherData.alt = 0;

  }

  

  // Format latitude and longitude in a string
  char gpsData[1000] = "";
  sprintf(gpsData, "\"lat\":%.6f, \"lng\":%.6f", weatherData.lat, weatherData.lng);

*/  




  // Ubidots
  if (!ubidots.connected()){                    // Reconnect to ubidots 
    ubidots.reconnect();
    ubidots.subscribe("/v2.0/devices/3/temperature/lv");
  }

  // triggers the routine every 5 seconds
  if (abs(millis() - ubiPubTS) > ubiPubFreq) 
  {
    char gpsData[1000] = "";
    
    // Format latitude and longitude in a string
    sprintf(gpsData, "\"lat\":%.6f, \"lng\":%.6f", gps.getLatitude(), gps.getLongitude());

    // Ubidots publish
    //          "Variable label"  "Value"         "Context"
    ubidots.add("gps",            1,              gpsData   );  // Generate / update GPS variable in Ubidots 
    ubidots.add("temperature",    ws.getTempC()             );  // Generate / update temperature variable in Ubidots
    ubidots.add("Humidity",       ws.getHum()               );  // Generate / update humidity variable in Ubidots
    ubidots.add("Pressure [hPa]", ws.getPressHPa()          );  // Generate / update pressure variable in Ubidots

    // ubidots.publish(DEVICE_LABEL);              // Publish buffer to Ubidots
    ubidots.publish("3");              // Publish buffer to Ubidots

    // Serial.println();
    // Serial.println("SUB: ");
    // Serial.println(ubidots.subscribe("/v2.0/devices/3/temperature/lv"));
    // Serial.println();

    ubidots.subscribe("/v2.0/devices/3/temperature/lv");

    ubiPubTS = millis();
  }
  ubidots.loop();

  // Recive incomming messages from RF module
  if(CCom.available(mt, msg))
  {
    Serial.println(mt);
    Serial.println(msg);
  }

}


void serialPrints()
{
    // Print position
  Serial.print("Current position:  ");
  Serial.print(gps.getLatitude(),6);
  Serial.print("N, ");
  Serial.print(gps.getLongitude(),6);
  Serial.println("E");
  
  // Print date
  Serial.print("Current date:      ");
  Serial.print(gps.getDay());
  Serial.print("/");
  Serial.print(gps.getMonth());
  Serial.print("-");
  Serial.println(gps.getYear());

  // Print time
  Serial.print("Current time:      ");
  Serial.print(gps.getHour());
  Serial.print(":");
  Serial.print(gps.getMinute());
  Serial.print(":");
  Serial.println(gps.getSecond());
  Serial.print("Time age:");
  Serial.println(gps.getTimeAge());

  Serial.println();

  // Print weather
  Serial.print("Current temp:      ");
  Serial.println(ws.getTempC());
  Serial.print("Current press:     ");
  Serial.println(ws.getPressHPa());
  Serial.print("Current hum:       ");
  Serial.println(ws.getHum());

  Serial.println();
  Serial.print("MQTT payload size: ");
  Serial.println(sizeof(callbackPayload));
  Serial.println();
}



/****************************************
 * Main Functions
 ****************************************/
// Setup
void setup() {
  
  // Start serial communication with microcontroller
  Serial.begin(serialBaud);                     
  Serial.println("Booting up");   

  // Ubidots 
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(UbisoftCallback);
  ubidots.setup();
  ubidots.reconnect();
  ubiPubTS = millis();      

  // ClusterCom / setup crypt key and device id
  CCom.begin(UBIDOTS_TOKEN, 1);     

  // Enable external sensors
  gps.enable();                                 // Power up GPS module and establish serial com 
  ws.enable();                                  // Establish I2C com with Weather Station
  CCom.enable();                                // Power up radio transmission modules
  
}


void loop(){
  
  // auxLoop();                                    // Loop all auxiliary utensils
  commLoop();                                   // Loop communication utensils
  serialPrints();                               // Run all desired prints
  display.showLockScreen();


  
}







