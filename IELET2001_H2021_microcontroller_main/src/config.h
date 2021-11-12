
#pragma once

#include <Arduino.h>



//////////////////////////////////////////////////////
//                                                  //
//                  DEFINE HARDWARE                 //
//                                                  //
//////////////////////////////////////////////////////

// I2C interface pins
const int i2cSDA =    21;
const int i2cSCL =    22;

// GPS
const int psGPS =    25;     // Power supply pin for GPS sensor ---------- SELECT OTHER PIN !
const int rxGPS =    37;     // Serial recieve pin GPS
const int txGPS =    38;     // Serial transmit pin GPS





//////////////////////////////////////////////////////
//                                                  //
//                    BAUD RATES                    //
//                                                  //
//////////////////////////////////////////////////////

// GPS
static const uint32_t serialBaud = 9600;
static const uint32_t GPSBaud = 9600;
