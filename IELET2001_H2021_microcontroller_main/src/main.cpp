
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "sensors_ext.h"
#include "config.h"

// GPS
TinyGPSPlus gps; // The TinyGPS++ object

// Software Serial 
// Because the hardware-based serial communication pins are busy, we create new by using the SoftwareSerial library

SoftwareSerial ssGPS(rxGPS, txGPS);            // Create software-based serial communication on pins


void setup() {
  
  Serial.begin(serialBaud);

  // Set baud rate for GPS communication
  ssGPS.begin(GPSBaud); // Serial communication with GPS module

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

