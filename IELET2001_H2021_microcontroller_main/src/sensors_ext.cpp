
#include "sensors_ext.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


// Constructor
GPS::GPS(int baud, int pinPS, int pinTx, int pinRx){
    GPSPsPin    = pinPS;
    GPSTxPin    = pinTx;
    GPSRxPin    = pinRx;
    GPSBaud     = baud;

    pinMode(GPSPsPin, OUTPUT);


    // Instanciate software serial communication to GPS
    SoftwareSerial ssGPS(GPSRxPin, GPSTxPin);

};


int GPS::getLatitude(){
    return 1;

};


// GPS sensor configuration

void GPS::setGPSPsPin(int pin){ 
    GPSPsPin = pin;
};

void GPS::setGPSRxPin(int pin){
    GPSRxPin = pin;
};

void GPS::setGPSTxPin(int pin){
    GPSTxPin = pin;
};

void GPS::enable(){

    digitalWrite(GPSPsPin, HIGH);
    Serial.println("Powering GPS sensor");







}