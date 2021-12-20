
#pragma once

#include <Arduino.h>

#define uS_TO_S_FACTOR 1000000ULL /* C onversion factor for micro seconds to seconds */
#define mS_TO_S_FACTOR 1000

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
const int gpsCheckHour1 =   19;         // First hour that GPS data should be obtained and sent to Ubidots
const int gpsCheckHour2 =   20;         // Second hour that GPS data should be obtained and sent to Ubidots
int       gpsReconAttempts = 5;         // Amount of attempts unit will try to recieve GPS data before giving up

// RF module
const int rxRF =            27;         // Serial recieve pin RF
const int txRF =            26;         // Serial transmit pin RF
const int psRFTX =          32;         // Power supply radio transmitter
const int psRFRX =          33;         // Power supply radio reciever
int fetchSlaveDataAttempts =5;          // Amount of times unit should try to fetch slave data 
int getIDAttempts =         5;          // Amount of times slave will try to get ID when awake


// Buttons
const int btnS1 =           0;          // HW pin for button 1
const int btnS2 =           35;         // HW pin for button 2

// Display
const int backLight =       4;          // HW pin for display backlight

// Battery surveilance
const int batteryPin =      36;         // HW pin for monitoring battery

// Set up eeprom storage
#define EEPROM_COMMIT 1                 // True because unit should save some data to EEPROM
#define EEPROM_SIZE 400                 // Size allocated to probram in EEPROM
#define SIZE_OF_MAC 18                  // Lenght of MAC-adress

#define CONFIG_STATE 2                  // Adress where Master / Slave / Not previously configured is stored
enum CONFIG : uint8_t
{
    NOT     = 255,
    MASTER  = 1,
    SLAVE   = 2
};

#define NUMB_OF_SLAVES_ADDRESS 10       // Max amount of slaves that can be associated with Master
#define ID_EEPROME_ADDRESS 100
#define MASTER_ID_EEPROME_ADDRESS 101
#define MAC_ADDRESS_SLAVE_START 200



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
//                      SETUP                       //
//                                                  //
//////////////////////////////////////////////////////


// Milliseconds given to user when selecting master/slave
int selTime =                       10 * mS_TO_S_FACTOR;

// Time to loop before attempting to get data from slaves
unsigned long timeWaitFetchSlaves = 240 * mS_TO_S_FACTOR;

// Maximum time used for trying to get data from slaves
unsigned long timeDataFetchSlaves = 120 * mS_TO_S_FACTOR;


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
const int ubiPubFreq =      2 * mS_TO_S_FACTOR;                         // Ubidots: Publish rate (ms between pub)
unsigned long ubiPubTS;                                                 // Ubidots: Timestamp for last publish


//////////////////////////////////////////////////////
//                                                  //
//                       WiFi                       //
//                                                  //
//////////////////////////////////////////////////////

// List of WiFi networks commonly used during development

const char *WIFI_SSID = "ClusterNet";                                   // WiFi : SSID General cluster network
const char *WIFI_PASS = "ClusterNet";                                   // WiFi : PW   General cluster network

#define LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS


//////////////////////////////////////////////////////
//                                                  //
//                   CLUSTERCOM                     //
//                                                  //
//////////////////////////////////////////////////////

#define ALLOWED_SLAVES 10                                               // Number of slaves allowed for each Master

//////////////////////////////////////////////////////
//                                                  //
//                       Sleep                      //
//                                                  //
//////////////////////////////////////////////////////


#define TIME_TO_SLEEP_CHECK 20 // How often to check if the contitions for sleep mode are met

