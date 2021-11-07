
#pragma once

#include <Arduino.h>



//////////////////////////////////////////////////////
//                                                  //
//                  DEFINE HARDWARE                 //
//                                                  //
//////////////////////////////////////////////////////

// I2C interface pins
static const int i2cSDA =    21;
static const int i2cSCL =    22;

// GPS
static const int rxGPS =    37;
static const int txGPS =    38;





//////////////////////////////////////////////////////
//                                                  //
//                    BAUD RATES                    //
//                                                  //
//////////////////////////////////////////////////////

// GPS
static const uint32_t serialBaud = 9600;
static const uint32_t GPSBaud = 9600;
