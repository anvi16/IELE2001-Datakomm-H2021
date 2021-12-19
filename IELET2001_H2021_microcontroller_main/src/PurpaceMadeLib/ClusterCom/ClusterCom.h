/*
 Name:		ClusterCom.h
 Created:	11/8/2021 12:46:13 PM
 Author:	
 Editor:	http://www.visualmicro.com
*/

#ifndef CLUSTER_COM_H
#define CLUSTER_COM_H

// Ensure ArduinoJSON Lib Intellisense works correctly
#define ARDUINOJSON_ENABLE_STD_STREAM 0

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <RH_ASK.h>
#include <RHEncryptedDriver.h>
#include <RHReliableDatagram.h>
#include <ArduinoJson.h>
#include <AES.h>
#include <SPI.h>
#include <WiFi.h>
#include <EEPROM.h>

#ifndef MAX_PACKET_SIZE
	#define MAX_PACKET_SIZE 47
#endif
#ifndef ID_EEPROME_ADDRESS
	#define ID_EEPROME_ADDRESS 100
#endif
#ifndef MASTER_ID_EEPROME_ADDRESS
	#define MASTER_ID_EEPROME_ADDRESS 101
#endif
#ifndef EEPROM_SIZE 
	#define EEPROM_SIZE 200
#endif
#ifndef DEBUG 
	#define DEBUG
#endif
#ifndef EEPROM_COMMIT
	#define EEPROM_COMMIT 1
#endif


class ClusterCom {

  public:

	// Message Type
    enum MT : uint8_t
    {
	    PING  = 1,
	    ID    = 2,
	    SLEEP = 3,
	    ERROR = 4,
	    DATA  = 5
    };

	ClusterCom(uint8_t pinRx = 27, uint8_t pinTx = 26, uint8_t pwrRx = 25, uint8_t pwrTx = 33, uint32_t serialBaud = 9600, uint8_t id = 0);

	void begin(const char* encryptkey = nullptr, uint16_t eepromSize = EEPROM_SIZE, uint16_t idEepromAddress = ID_EEPROME_ADDRESS, uint16_t masterIdEepromAddress = MASTER_ID_EEPROME_ADDRESS);
	bool send(const char* msg, uint8_t receiver, MT mt, uint8_t id = 0);
	bool send(float msg, uint8_t receiver, MT mt, uint8_t id = 0);
	bool available(uint8_t *mt, String *msgStr, float *msgFloat, uint8_t* id);
	bool available(uint8_t* mt, String* msgStr, float* msgFloat, uint8_t* id, uint16_t wait);
	bool getId();
	void setId(uint8_t id, bool storeInEeprom = false);
	void enable();
	void disable();

	uint8_t masterId = 1; // If connected to master

  private:
  	bool send(float msgFloat, const char* msgStr, uint8_t receiver = 1, MT mt = DATA, uint8_t id = 0);
	void readRecivedData(uint8_t *mt, String *msgStr, float *msgFlot, uint8_t *id = nullptr);
	bool reciveId(String mac);
	

	// Instantiate ASK communication 
	RH_ASK rfCom;
	// Instantiate AES128 block ciphering
	AES128 aes128;   
	// The RHEncryptedDriver acts as a wrapper for the actual radio driver
	RHEncryptedDriver driver;
	// Class to manage message delivery and receipt, using the driver declared above
	RHReliableDatagram manager;

	// The key MUST be the same on all devices
	unsigned char _encryptkey[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };

	StaticJsonDocument<MAX_PACKET_SIZE> Json_Buffer;

	uint8_t _buf[RH_ASK_MAX_MESSAGE_LEN];
	uint8_t messageType;
	String message;

	uint32_t _serialBaud;
	uint8_t _pinRx; // Serial interrupt pin to receive data
	uint8_t _pinTx; // Serial pin to transmitt data
	uint8_t _pwrRx; // Power pin, enable/disable  RF reciver module
	uint8_t _pwrTx; // Power pin, enable/disable RF transmitter module
	uint8_t _id; // Device id

	uint16_t _idEepromAddress;
	uint16_t _masterIdEepromAddress;

};
#endif
