
#pragma once

#include <Arduino.h>

// Uncomment to enable debug in program


//////////////////////////////////////////////////////
//                                                  //
//                  DEFINE HARDWARE                 //
//                                                  //
//////////////////////////////////////////////////////

// I2C interface pins
const int i2cSDA =          21;         // Green wire
const int i2cSCL =          22;         // Yellow wire
const int i2cBME280Adr =    0x76;       // BME280 adress

// GPS
const int psGPS =           17;         // Power supply pin for GPS sensor
const int rxGPS =           13;         // Serial recieve pin GPS
const int txGPS =           12;         // Serial transmit pin GPS

// RF module
const int rxRF =            27;         // Serial recieve pin RF
const int txRF =            26;         // Serial transmit pin RF
const int psRFTX =          32;         // Power supply radio transmitter
const int psRFRX =          33;         // Power supply radio reciever


// Buttons
const int btnS1 =           0;
const int btnS2 =           35;

// Display
const int backLight =       4;

// Battery surveilance
const int batteryPin =      36;

// Set up eeprom storage
#define EEPROM_SIZE 200
#define NUMB_OF_SLAVES_ADDRESS 10
#define ID_EEPROME_ADDRESS 100
#define MASTER_ID_EEPROME_ADDRESS 101


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

const char *WIFI_SSID = "ClusterNet";                                   // WiFi : SSID General cluster network
const char *WIFI_PASS = "ClusterNet";                                   // WiFi : PW   General cluster network



//////////////////////////////////////////////////////
//                                                  //
//                       Sleep                      //
//                                                  //
//////////////////////////////////////////////////////

#define uS_TO_S_FACTOR 1000000 /* C onversion factor for micro seconds to seconds */
#define mS_TO_S_FACTOR 1000
#define TIME_TO_SLEEP 6000       /* Time ESP32 will go to sleep (in seconds) */
#define TIME_TO_SLEEP_CHECK 6000 // How often to check if the contitions for sleep mode are met

