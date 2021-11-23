/*
 Name:		ClusterCom.h
 Created:	11/8/2021 12:46:13 PM
 Author:	Andreas Vik
 Editor:	http://www.visualmicro.com
*/

#ifndef CLUSTER_COM_H
#define CLUSTER_COM_H

#include "Arduino.h"
#include <stdint.h>
#include "lib/RadioHead-master/RH_ASK.h"
#include "lib/RadioHead-master/RHEncryptedDriver.h"
#include "lib/RadioHead-master/RHReliableDatagram.h"
#include "lib/ArduinoJson-6.x/ArduinoJson.h"
#include "lib/Crypto/src/AES.h"
#include <SPI.h>

// Ensure ArduinoJSON Lib Intellisense works correctly
#define ARDUINOJSON_ENABLE_STD_STREAM 0

#define MAX_PACKET_SIZE 47
//#define DEBUG


class ClusterCom {

  public:

	// Message Type
    enum MT : uint8_t
    {
	    PING  = 1,
	    TIME  = 2,
	    SLEEP = 3,
	    ERROR = 4,
	    DATA  = 5
    };

	ClusterCom(uint8_t id = 0, uint32_t serialBaud = 9600, uint8_t pinRx = 27, uint8_t pinTx = 26);

	void begin(const char* encryptkey = nullptr, uint8_t id = 0);
	bool send(uint8_t receiver, const char* msg = nullptr, MT mt = DATA);
	bool available();
	String output();

  private:
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

	uint8_t _buf[RH_ASK_MAX_MESSAGE_LEN];

	uint32_t _serialBaud;
	uint8_t _pinRx; // Serial interrupt pin to receive data
	uint8_t _pinTx; // Serial pin to transmitt data
	uint8_t _id; // Device id

};
#endif