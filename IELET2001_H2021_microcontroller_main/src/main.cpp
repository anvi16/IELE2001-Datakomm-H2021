
// Include libraries
#include <Arduino.h>                            // General Arduino C++ lib
#include <TinyGPS++.h>                          // GPS lib for easy extraction of GPS data
#include <SoftwareSerial.h>                     // Software serial lib (for serial com on any pin)
#include <time.h>                               // Time lib Arduino
#include <cstdio>
#include <iostream>                            
#include <TFT_eSPI.h>                           // TFT lib for control of OLED screen
#include <SPI.h>                                // SPI lib. TFT lib depends on this
#include <Wire.h>

// Include files with dependencies
#include "sensors_ext.h"                        // File containging classes related to peripheral sensors (GPS, temp, hum, press)
#include "config.h"                             // Config file for HW pins, baud etc.

// TFT class for OLED display control
TFT_eSPI tft = TFT_eSPI(135, 240);

// GPS class - Constructor takes GPS module baud rate and pins for PS, Tx and Rx
GPS gps(GPSBaud, serialBaud, psGPS, txGPS, rxGPS);

// Weather station class (for simple reading of weather station data)
WeatherStation ws(i2cBME280Adr);

// Ubidots object
Ubidots ubidots(UBIDOTS_TOKEN);


/****************************************
 * Auxiliar Functions
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
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

// Setup
void setup() {
  
  // Start serial communication with microcontroller
  Serial.begin(serialBaud);                     
  Serial.println("Booting up");   

  // Ubidots 
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubiPubTS = millis();           

  // Enable external sensors
  gps.enable();                                 // Power up GPS module and establish serial com 
  ws.enable();                                  // Establish I2C com with Weather Station
}


void loop(){

  gps.refresh();                                // Update data from GPS

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_GREEN);
  tft.print("Test");

  // Ubidots
  if (!ubidots.connected()){                    // Reconnect to ubidots 
    ubidots.reconnect();}

  if (abs(millis() - ubiPubTS) > ubiPubFreq) // triggers the routine every 5 seconds
  {
    char gpsData[1000] = "";
    
    // Format latitude and longitude in a string
    sprintf(gpsData, "\"lat\":%.6f, \"lng\":%.6f", gps.getLatitude(), gps.getLongitude());

    // Ubidots publish
    //          "Variable label"  "Value"         "Context"
    ubidots.add("gps",            1,              gpsData   );  // Generate / update GPS variable in Ubidots 
    ubidots.add("Temperature",    ws.getTempC()             );  // Generate / update temperature variable in Ubidots
    ubidots.add("Humidity",       ws.getHum()               );  // Generate / update humidity variable in Ubidots
    ubidots.add("Pressure [hPa]", ws.getPressHPa()          );  // Generate / update pressure variable in Ubidots

    ubidots.publish(DEVICE_LABEL);              // Publish buffer to Ubidots

    ubiPubTS = millis();
  }


  ubidots.loop();

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

  delay(0);

}













