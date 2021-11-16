
#pragma once

#include <Arduino.h>



//////////////////////////////////////////////////////
//                                                  //
//                  DEFINE HARDWARE                 //
//                                                  //
//////////////////////////////////////////////////////

// I2C interface pins
const int i2cSDA =          21;   // Green wire
const int i2cSCL =          22;   // Yellow wire
const int i2cBME280Adr =    0x76; // BME280 adress

// GPS
const int psGPS =    25;          // Power supply pin for GPS sensor ---------- SELECT OTHER PIN !
const int rxGPS =    37;          // Serial recieve pin GPS
const int txGPS =    38;          // Serial transmit pin GPS


//////////////////////////////////////////////////////
//                                                  //
//                    BAUD RATES                    //
//                                                  //
//////////////////////////////////////////////////////

// GPS
static const uint32_t serialBaud = 9600;
static const uint32_t GPSBaud = 9600;





//////////////////////////////////////////////////////
//                                                  //
//                     UBIDOTS                      //
//                                                  //
//////////////////////////////////////////////////////

// Based on example https://help.ubidots.com/en/articles/748067-connect-an-esp32-devkitc-to-ubidots-over-mqtt

// Library dependencies

#include <UbidotsEsp32Mqtt.h>

/****************************************
 * Define Constants
 ****************************************/
const char *UBIDOTS_TOKEN = "BBFF-pzGZkRPVMO7baDRylwiST623Ow5DrF";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "Inteno-D81D";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "XJRBC6IITC42CO";   // Put here your Wi-Fi password
const char *DEVICE_LABEL = "Aalesund";       // Put here your Device label to which data will be published
const char *VARIABLE_LABEL = "Temperature";        // Put here your Variable label to which data  will be published

const int PUBLISH_FREQUENCY = 5000;         // Update rate in milliseconds

unsigned long timer;


