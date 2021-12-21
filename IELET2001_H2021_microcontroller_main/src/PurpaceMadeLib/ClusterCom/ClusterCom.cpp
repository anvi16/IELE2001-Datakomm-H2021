/*
 Name:		ClusterCom.cpp
 Created:	11/8/2021 12:46:13 PM
 Author:	
 Editor:	http://www.visualmicro.com
*/

#include "ClusterCom.h"


ClusterCom::ClusterCom(uint8_t pinRx, uint8_t pinTx, uint8_t pwrRx, uint8_t pwrTx, uint32_t serialBaud, uint8_t id) :
	rfCom(2000, pinRx, pinTx, pwrTx),
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
void ClusterCom::begin(const char* encryptkey, uint16_t eepromSize, uint16_t idEepromAddress)
{
    _idEepromAddress = idEepromAddress;

    if(!Serial) Serial.begin(_serialBaud);
    pinMode(_pwrTx,OUTPUT);     // Set pinmode for control of power supply transmitter
    pinMode(_pwrRx,OUTPUT);     // Set pinmode for control of power supply reciever
    
    manager.init();
    manager.setTimeout(1500);

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
    return send(0, msg, receiver, mt, id);
}


bool ClusterCom::send(float msg, uint8_t receiver, MT mt, uint8_t id)
{
    return send(msg, NULL, receiver, mt, id);
}


bool ClusterCom::send(float msgFloat, const char* msgStr, uint8_t receiver, MT mt, uint8_t id)
{
    const char* msgBuf = msgStr;

    Json_Buffer.clear();
    if(id)       Json_Buffer["id"]  = id;
                 Json_Buffer["mt"]  = mt;
	if(msgBuf)   Json_Buffer["str"] = msgBuf;
    if(msgFloat) Json_Buffer["flo"] = msgFloat;

    char buffer[MAX_PACKET_SIZE];
    size_t n = serializeJson(Json_Buffer, buffer);

	#ifdef DEBUG
		Serial.print("Package size Json : ");
		Serial.print(n);
        Serial.print("  :  ");
        Serial.print(mt);
        Serial.print(" , ");
        if(msgBuf)   Serial.println(msgBuf);
        if(msgFloat) Serial.println(msgFloat);
	#endif

    if (n <= MAX_PACKET_SIZE) {
        // Send message
        if(manager.sendtoWait((uint8_t*)buffer, n, receiver)) 
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


bool ClusterCom::available(uint8_t* mt, String* msgStr, float* msgFloat, uint8_t* id)
{
    available(mt, msgStr, msgFloat, id, 0);
}


bool ClusterCom::available(uint8_t* mt, String* msgStr, float* msgFloat, uint8_t* id, uint16_t wait)
{
    uint8_t from;
    uint8_t len = sizeof(_buf);

    // Wait for a message addressed to client
    if(wait) 
    {
        if (!manager.recvfromAckTimeout(_buf, &len, wait, &from)) return false;
    }
    else   
    {  
        if(manager.available())
            if (!manager.recvfromAck(_buf, &len, &from)) return false;
    }
    
    #ifdef DEBUG
        Serial.print("got request from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)_buf);
    #endif
            
    *id = from;
    readRecivedData(mt, msgStr, msgFloat);
    return true;
}



bool ClusterCom::reciveId(String mac)
{
    uint8_t from;
    uint16_t wait = 1500*3;   // Wait max timeout time for acknowledgement and numb of retransmits
    uint8_t len = sizeof(_buf);
	
    // Wait for a message addressed to client
    if (manager.recvfromAckTimeout(_buf, &len, wait, &from))
    {
        #ifdef DEBUG
            Serial.print("From: ");
            Serial.print(from);
            Serial.print(" : ");
            Serial.println((char*)_buf);
        #endif

        uint8_t mt;
        String msgStr;
        float  msgflot;
        uint8_t id;

        readRecivedData(&mt, &msgStr, &msgflot, &id);

        if(msgStr == mac && mt == ID)
        {
            #ifdef DEBUG
                Serial.print("ID resived: ");
                Serial.println(id);
            #endif

            masterId = from;
            setId(id, true);
            return true;
        }
    }
    return false;
}


bool ClusterCom::getId()
{
    if (0 < EEPROM.read(_idEepromAddress) && EEPROM.read(_idEepromAddress) < 255)
    {
        #ifdef DEBUG
            Serial.println("Using stored id");
        #endif

        setId(EEPROM.read(_idEepromAddress));
        return true;
    }

    int8_t retry = 0;
    String mac = WiFi.macAddress();

    if(!send(mac.c_str(), masterId, ID)) return false;

    while(!reciveId(mac)) 
    {
        if(--retry <= 0) 
            return false;

        send(mac.c_str(), masterId, ID);
    }
    return true;
}


void ClusterCom::setId(uint8_t id, bool storeInEeprom)
{
    _id = id;
    manager.setThisAddress(id);
    #ifdef DEBUG
        Serial.print("Address set to: ");
        Serial.println(id);
    #endif

    // Store id in flash memory
    if (storeInEeprom)
    {
        EEPROM.write(id, _idEepromAddress);
        if(EEPROM_COMMIT) EEPROM.commit();
    }
}


void ClusterCom::readRecivedData(uint8_t *mt, String *msgStr, float *msgFloat, uint8_t *id)
{
    Json_Buffer.clear();
	deserializeJson(Json_Buffer, _buf, sizeof(_buf));

    uint8_t mtBuf       = Json_Buffer["mt"];
    String  msgStrBuf   = Json_Buffer["str"];
    float   msgFloatBuf = Json_Buffer["flo"];
    uint8_t idBuf       = Json_Buffer["id"];

    if (mt)       *mt       = mtBuf;
    if (msgStr)   *msgStr   = msgStrBuf;
    if (msgFloat) *msgFloat = msgFloatBuf;
    if (id)       *id       = idBuf;  
}


void ClusterCom::enable(){
    #ifdef DEBUG
        Serial.println("Enabling radio transmission modules");
    #endif
    //digitalWrite(_pwrTx, HIGH);               // Powering up radio transmission module / Handeled by Radiohead lib
    digitalWrite(_pwrRx, HIGH);               // Powering up radio reciever module
    #ifdef DEBUG
        Serial.println("Radio transmission modules powered up");
    #endif

}


void ClusterCom::disable(){
    #ifdef DEBUG
        Serial.println("Disabling radio transmission modules");
    #endif
    //digitalWrite(_pwrTx, LOW);               // Powering down radio transmission module, / Handeled by Radiohead lib
    digitalWrite(_pwrRx, LOW);               // Powering down radio reciever module
    #ifdef DEBUG
        Serial.println("Radio transmission modules powered down");
    #endif

}
