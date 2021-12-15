/*
 Name:		ClusterCom.cpp
 Created:	11/8/2021 12:46:13 PM
 Author:	Andreas Vik
 Editor:	http://www.visualmicro.com
*/

#include "ClusterCom.h"


ClusterCom::ClusterCom(uint8_t pinRx, uint8_t pinTx, uint8_t pwrRx, uint8_t pwrTx, uint32_t serialBaud, uint8_t id) :
	rfCom(2000, pinRx, pinTx, 0),
    driver(rfCom, aes128),
    manager(driver, id)
{
	_id = id;
	_serialBaud = serialBaud;
    _pinRx = pinRx;
    _pinTx = pinTx;
    _pwrRx = pwrRx;
    _pwrTx = pwrTx;
	
}

// Encryption string must be 16 + 5 prefix char long
void ClusterCom::begin(const char* encryptkey, uint16_t eepromSize, uint16_t idEepromAddress, uint16_t masterIdEepromAddress)
{
    _idEepromAddress = idEepromAddress;
    _masterIdEepromAddress = idEepromAddress;

    Serial.begin(_serialBaud);
    pinMode(_pwrTx,OUTPUT);     // Set pinmode for control of power supply transmitter
    pinMode(_pwrRx,OUTPUT);     // Set pinmode for control of power supply reciever

    manager.init();

    EEPROM.begin(eepromSize);

    // Now set up the encryption key in cipher
    if (encryptkey != nullptr) 
    {
        size_t n = sizeof(_encryptkey)/sizeof(_encryptkey[0]);

        // Copy n elements of user choosen encryptkey to internall storage
        for (int i=0; i < n; i++){
            _encryptkey[i] = encryptkey[i];
        }
        aes128.setKey(_encryptkey, sizeof(_encryptkey));
    }
    else
        // Use predefined encryptkey to library
        aes128.setKey(_encryptkey, sizeof(_encryptkey));
}

bool ClusterCom::send(const char* msg, uint8_t receiver, MT mt, uint8_t id)
{
    StaticJsonDocument<MAX_PACKET_SIZE> Json_Buffer;
    Json_Buffer["id"]   = id;
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


bool ClusterCom::available(uint8_t &mt, String &msg)
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

            mt = messageType();
            msg = message();

            return true;
        }
    }
    return false;
}


bool ClusterCom::reciveId(const char* mac)
{
    uint8_t from;
    uint16_t wait = 2000; // 2sek
    uint8_t len = sizeof(_buf);

    StaticJsonDocument<MAX_PACKET_SIZE> Json_Buffer;
	
    if (manager.available())
    {
        // Wait for a message addressed to client
        if (manager.recvfromAckTimeout(_buf, &len, wait, &from))
        {
            deserializeJson(Json_Buffer, _buf, sizeof(_buf));
            uint8_t id = Json_Buffer["id"];
            uint8_t mt = Json_Buffer["mt"];
            const char* msg = Json_Buffer["msg"];

            if(strcmp(msg, mac) == 0 && mt == ID)
            {
                masterId = from;
                setId(id, true);
                Serial.print("ID resived: ");
                Serial.println(id);
                return true;
            }
        }
    }
    return false;
}




bool ClusterCom::getId()
{
    
    /* if (EEPROM.read(_idEepromAddress))
    {
        setId(EEPROM.read(_idEepromAddress));
        return true;
    } */
    
    //Serial.println(EEPROM.read(_idEepromAddress));

    int8_t retry = 2;
    const char* mac = WiFi.macAddress().c_str();

    while(!send(mac, masterId, ID) && !reciveId(mac))
    {
        Serial.print("LOOP: ");
        Serial.println(mac);
        if(--retry <= 0) return false;
    }
    return true;
}


void ClusterCom::setId(uint8_t id, bool storeInEeprom)
{
    _id = id;
    manager.setThisAddress(id);

    // Store id in flash memory
    if (storeInEeprom)
    {
        EEPROM.write(id, _idEepromAddress);
        //EEPROM.commit();
    }
}


String ClusterCom::message()
{
    StaticJsonDocument<MAX_PACKET_SIZE> Json_Buffer;
	deserializeJson(Json_Buffer, _buf, sizeof(_buf));

	return Json_Buffer["msg"];
}


uint8_t ClusterCom::messageType()
{
    StaticJsonDocument<MAX_PACKET_SIZE> Json_Buffer;
	deserializeJson(Json_Buffer, _buf, sizeof(_buf));

	return (uint8_t)Json_Buffer["mt"];
}

void ClusterCom::enable(){
    #ifdef DEBUG
        Serial.println("Enabling radio transmission modules");
    #endif
    digitalWrite(_pwrTx, HIGH);               // Powering up radio transmission module
    digitalWrite(_pwrRx, HIGH);               // Powering up radio reciever module
    #ifdef DEBUG
        Serial.println("Radio transmission modules powered up");
    #endif

}

void ClusterCom::disable(){
    #ifdef DEBUG
        Serial.println("Disabling radio transmission modules");
    #endif
    digitalWrite(_pwrTx, LOW);               // Powering down radio transmission module
    digitalWrite(_pwrRx, LOW);               // Powering down radio reciever module
    #ifdef DEBUG
        Serial.println("Radio transmission modules powered down");
    #endif

}
