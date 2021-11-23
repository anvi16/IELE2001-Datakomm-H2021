/*
 Name:		ClusterCom.cpp
 Created:	11/8/2021 12:46:13 PM
 Author:	Andreas Vik
 Editor:	http://www.visualmicro.com
*/

#include "ClusterCom.h"


ClusterCom::ClusterCom(uint8_t id, uint32_t serialBaud, uint8_t pinRx, uint8_t pinTx) :
	rfCom(2000, pinRx, pinTx, 0),
    driver(rfCom, aes128),
    manager(driver, id)
{
	_id = id;
	_serialBaud = serialBaud;
    _pinRx = pinRx;
    _pinTx = pinTx;
}

// Encryption string must be 16 + 5 prefix char long
void ClusterCom::begin(const char* encryptkey, uint8_t id)
{
    Serial.begin(_serialBaud);

    // Set device id after constructor
    _id = id;
    manager.setThisAddress(id);
    manager.init();

    // Now set up the encryption key in cipher
    if (encryptkey != nullptr) 
    {
        size_t n = sizeof(_encryptkey)/sizeof(_encryptkey[0]);

        // Copy n elements of user choosen encryptkey to internall storage
        for (int i=0; i < n; i++){
            _encryptkey[i] = encryptkey[i+5];
        }
        aes128.setKey(_encryptkey, sizeof(_encryptkey));
    }
    else
        // Use predefined encryptkey to library
        aes128.setKey(_encryptkey, sizeof(_encryptkey));
}

bool ClusterCom::send(uint8_t receiver, const char* msg, MT mt)
{

    StaticJsonDocument<MAX_PACKET_SIZE> Json_Buffer;
    Json_Buffer["mt"]   = mt;
	Json_Buffer["msg"]  = msg;

    char buffer[MAX_PACKET_SIZE];
    size_t n = serializeJson(Json_Buffer, buffer);

	#ifdef DEBUG
		Serial.print("Package size Json : ");
		Serial.println(n);
	#endif

    if (n <= MAX_PACKET_SIZE) {
        // Send message
        if (manager.sendtoWait((uint8_t*)buffer, n, receiver)) 
        {
            #ifdef DEBUG
                Serial.println("ACK");
            #endif

            return true;
        }
        #ifdef DEBUG
            else Serial.println("sendtoWait failed");
        #endif
    }
    #ifdef DEBUG
        else 
        {
            Serial.println("------ERROR-------");
            Serial.println("Message is to large, try to increase \"MAX_PACKET_SIZE\" ");
            Serial.print("Tried to send: ");
            Serial.print(n);
            Serial.print("bytes");
        }
    #endif

    return false;
}


bool ClusterCom::available()
{
    uint8_t from;
    uint8_t len = sizeof(_buf);

    if (manager.available())
    {
        // Wait for a message addressed to client
        if (manager.recvfromAck(_buf, &len, &from))
        {
			#ifdef DEBUG
	            Serial.print("got request from : 0x");
	            Serial.print(from, HEX);
	            Serial.print(": ");
	            Serial.println((char*)_buf);
            #endif

            return true;
        }
    }
    return false;
}

String ClusterCom::output()
{
    StaticJsonDocument<MAX_PACKET_SIZE> Json_Buffer;
	deserializeJson(Json_Buffer, _buf, sizeof(_buf));

	return Json_Buffer["msg"];
}


