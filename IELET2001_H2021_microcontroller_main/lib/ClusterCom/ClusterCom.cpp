/*
 Name:		ClusterCom.cpp
 Created:	11/8/2021 12:46:13 PM
 Author:	Andreas Vik
 Editor:	http://www.visualmicro.com
*/

#include "ClusterCom.h"


ClusterCom::ClusterCom(uint8_t id, uint32_t serialBaud, uint8_t pin_rx, uint8_t pin_tx) :
	rf_com(2000, pin_rx, pin_tx, 0),
    driver(rf_com, aes128),
    manager(driver, id)
{
	_id = id;
	_serialBaud = serialBaud;
    _pin_rx = pin_rx;
    _pin_tx = pin_tx;
}

void ClusterCom::begin()
{
    Serial.begin(_serialBaud);
    manager.init();

    // Now set up the encryption key in our cipher
    aes128.setKey(encryptkey, sizeof(encryptkey));
}

void ClusterCom::send(uint8_t receiver, const char* msg, MT mt)
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

    try {
        if (n <= MAX_PACKET_SIZE) {
            // Send message
            if (!manager.sendtoWait((uint8_t*)buffer, n, receiver)) Serial.println("sendtoWait failed");
			#ifdef DEBUG
				else Serial.println("ACK");
			#endif
        }
        else {
            throw n;
        }
    }
    catch (int n) {
		#ifdef DEBUG
	        Serial.println("------ERROR-------");
	        Serial.println("Message is to large, try to increase \"MAX_PACKET_SIZE\" ");
	        Serial.print("Tried to send: ");
	        Serial.print(n);
	        Serial.print("bytes");
		#endif
    }
}


bool ClusterCom::available()
{
    uint8_t from;
    uint8_t len = sizeof(_buf);

    if (manager.available())
    {
        // Wait for a message addressed to us from the client
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


