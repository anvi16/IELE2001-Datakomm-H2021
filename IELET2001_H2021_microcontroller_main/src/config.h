
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
const int psGPS =    25;          // Power supply pin for GPS sensor
const int rxGPS =    37;          // Serial recieve pin GPS
const int txGPS =    38;          // Serial transmit pin GPS


//////////////////////////////////////////////////////
//                                                  //
//                    BAUD RATES                    //
//                                                  //
//////////////////////////////////////////////////////

// Serial communication
static const uint32_t serialBaud = 9600;

// GPS
static const uint32_t GPSBaud = 9600;


//////////////////////////////////////////////////////
//                                                  //
//                     UBIDOTS                      //
//                                                  //
//////////////////////////////////////////////////////

// Based on example https://help.ubidots.com/en/articles/748067-connect-an-esp32-devkitc-to-ubidots-over-mqtt

// Library dependencies
#include <UbidotsEsp32Mqtt.h>

// Publish variables
const char *UBIDOTS_TOKEN = "BBFF-pzGZkRPVMO7baDRylwiST623Ow5DrF";      // Token corresponding to IELET2001 2021 gr30
const char *DEVICE_LABEL = "Aalesund";                                  // Ubidots: Device label to which data will be published
const int ubiPubFreq = 5000;                                            // Ubidots: Publish rate (ms between pub)
unsigned long ubiPubTS;                                                 // Ubidots: Timestamp for last publish


//////////////////////////////////////////////////////
//                                                  //
//                       WiFi                       //
//                                                  //
//////////////////////////////////////////////////////

// List of WiFi networks commonly used during development

// const char *WIFI_SSID = "Inteno-D81D";                               // WiFi : SSID Latinskolegata 1, Aalesund
// const char *WIFI_PASS = "XJRBC6IITC42CO";                            // WiFi : PW   Latinskolegata 1, Aalesund

const char *WIFI_SSID = "Playboy Penthouse Solsiden";                   // WiFi : SSID Dyre Halses gate 4, Trondheim
const char *WIFI_PASS = "hughhefner";                                   // WiFi : PW   Dyre Halses gate 4, Trondheim
