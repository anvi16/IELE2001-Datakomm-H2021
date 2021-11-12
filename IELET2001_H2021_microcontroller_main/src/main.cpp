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

// Functionaly the same as the "delay()" function. D
void espDelay(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

void setup() {
  
  Serial.begin(9600);
  pinMode(LEDPin, OUTPUT);

  // Set baud rate for GPS communication
  ssGPS.begin(GPSBaud); // Serial communication with GPS module

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_GREEN);
  tft.print("Test");
}




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

  Serial.println();
  Serial.println();
  delay(1000);
}



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

  
}


/*
void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ssGPS.available() > 0){
    gps.encode(ssGPS.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
    }
  }
}
*/

  /*
  // Displays information when new sentence is available.
  while (ssGPS.available() > 0){
    Serial.write(ssGPS.read());
  }
    
  if (gps.location.isValid()) {
    double latitude = (gps.location.lat());
    double longitude = (gps.location.lng());
    Serial.println("Lat");
    Serial.println(latitude);
    Serial.println("Long");
    Serial.println(longitude);
  }
  */

  // GPS displayInfo

