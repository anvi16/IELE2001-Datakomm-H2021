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

#define MAX_PACKET_SIZE 47
//#define DEBUG


class ClusterCom {

  public:

    enum MT : uint8_t
    {
	    PING  = 1,
	    TIME  = 2,
	    SLEEP = 3,
	    ERROR = 4,
	    DATA  = 5
    };

	ClusterCom(uint8_t id, uint32_t serialBaud = 9600, uint8_t pin_rx = 26, uint8_t pin_tx = 27);

	void begin();
	void send(uint8_t receiver, const char* msg = nullptr, MT mt = DATA);
	bool available();
	String output();

  private:

	RH_ASK rf_com;
	// You can choose any of several encryption ciphers
	AES128 aes128;   // Instantiate AES128 block ciphering
	// The RHEncryptedDriver acts as a wrapper for the actual radio driver
	RHEncryptedDriver driver;
	// Class to manage message delivery and receipt, using the driver declared above
	RHReliableDatagram manager;
	// The key MUST be the same as the one in the server
	unsigned char encryptkey[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };

	uint8_t _buf[RH_ASK_MAX_MESSAGE_LEN];

	uint32_t _serialBaud;
	uint8_t _pin_rx; // interrupt pin to receive data
	uint8_t _pin_tx;
	uint8_t _id; // device id

};
#endif