<<<<<<< Updated upstream
#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "sensors_ext.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>

TFT_eSPI tft = TFT_eSPI(135, 240);

// GPS
TinyGPSPlus gps; // The TinyGPS++ object

// Software Serial 
// Because the hardware-based serial communication pins are busy, we create new by using the SoftwareSerial library
static const int RXPin =    21;
static const int TXPin =    22;
static const uint32_t GPSBaud = 9600;
SoftwareSerial ssGPS(RXPin, TXPin);            // Create software-based serial communication on pins

const int LEDPin =          12;
=======

// Include libraries
#include <TinyGPS++.h>
#include <SoftwareSerial.h>                     // Software serial library
#include <time.h>                               // Time library Arduino

#include <cstdio>
#include <iostream>

// Include files with dependencies
#include "sensors_ext.h"                        // File containging classes related to peripheral sensors (GPS, temp, hum, press)
#include "config.h"                             // Config file for HW pins, baud etc.

// GPS class - Constructor takes GPS module baud rate and pins for PS, Tx and Rx
GPS gps(GPSBaud, serialBaud, psGPS, txGPS, rxGPS);
WeatherStation ws(i2cBME280Adr);

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


>>>>>>> Stashed changes

// Functionaly the same as the "delay()" function. The diffrence is that instead of just waiting the ESP enters light sleep.This reduces the power consumption compared to the regular delay function

void espDelay(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

// Setup
void setup() {
  
<<<<<<< Updated upstream
  Serial.begin(9600);
  pinMode(LEDPin, OUTPUT);
=======
  // Start serial communication with microcontroller
  Serial.begin(serialBaud);                     
  Serial.println("Booting up");      

  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  timer = millis();           

  // Enable external sensors
  gps.enable();                                 // Power up GPS module and establish serial com 
  ws.enable();                                  // Establish I2C com with Weather Station

}


void loop(){
>>>>>>> Stashed changes

  gps.refresh();                                // Update data from GPS

<<<<<<< Updated upstream
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_GREEN);
}
=======
  // Ubidots
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
>>>>>>> Stashed changes

  if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    char gpsData[1000] = "";
    
    // Format latitude and longitude in a string
    sprintf(gpsData, "\"lat\":%.6f, \"lng\":%.6f", gps.getLatitude(), gps.getLongitude());

    // Ubidots publish
    //          "Variable"        "Value"         "Context"
    ubidots.add("gps",            1,              gpsData   );  // Generate / update GPS variable in Ubidots 
    ubidots.add("Temperature",    ws.getTempC()             );  // Generate / update Temperature variable in Ubidots
    ubidots.add("Humidity",       ws.getHum()               );  // Generate / update Temperature variable in Ubidots
    ubidots.add("Pressure [hPa]", ws.getPressHPa()          );  // Generate / update Temperature variable in Ubidots

    ubidots.publish(DEVICE_LABEL);              // Publish buffer to Ubidots

<<<<<<< Updated upstream
void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }
=======
    Serial.println(gpsData);                    // Print context (debug)

    timer = millis();
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
>>>>>>> Stashed changes

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



<<<<<<< Updated upstream
void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ssGPS.available() > 0){
    if (gps.encode(ssGPS.read())){
      displayInfo();
      byte gpsData = ssGPS.read();
      Serial.println(gpsData);
    }
  }
  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while(true);
  }
=======
>>>>>>> Stashed changes




























