
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "sensors_ext.h"
#include "config.h"



// GPS - homemade

GPS gps(GPSBaud, psGPS, txGPS, rxGPS);



// GPS
TinyGPSPlus gpsx; // The TinyGPS++ object



// Software Serial 
// Because the hardware-based serial communication pins are busy, we create new by using the SoftwareSerial library

SoftwareSerial ssGPS(rxGPS, txGPS);            // Create software-based serial communication on pins


void setup() {
  
  // Start serial communication with microcontroller
  Serial.begin(serialBaud);

  // GPS - Configure hardware for GPS sensor
  gps.setGPSPsPin(psGPS);
  gps.setGPSRxPin(rxGPS);
  gps.setGPSTxPin(txGPS);

  







  // Set baud rate for GPS communication
  ssGPS.begin(GPSBaud); // Serial communication with GPS module

}







void displayInfo()
{
  if (gpsx.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gpsx.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gpsx.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gpsx.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gpsx.date.isValid())
  {
    Serial.print(gpsx.date.month());
    Serial.print("/");
    Serial.print(gpsx.date.day());
    Serial.print("/");
    Serial.println(gpsx.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gpsx.time.isValid())
  {
    if (gpsx.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gpsx.time.hour());
    Serial.print(":");
    if (gpsx.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gpsx.time.minute());
    Serial.print(":");
    if (gpsx.time.second() < 10) Serial.print(F("0"));
    Serial.print(gpsx.time.second());
    Serial.print(".");
    if (gpsx.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gpsx.time.centisecond());
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
    if (gpsx.encode(ssGPS.read())){
      displayInfo();
      byte gpsData = ssGPS.read();
      Serial.println(gpsData);
    }
  }
  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gpsx.charsProcessed() < 10)
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

