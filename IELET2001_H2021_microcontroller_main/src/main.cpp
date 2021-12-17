
#include "config.h"                             // Config file for HW pins, baud etc.

// Include libraries
#include <Arduino.h>                            // General Arduino C++ lib
#include <TinyGPS++.h>                          // GPS lib for easy extraction of GPS data
#include <SoftwareSerial.h>                     // Software serial lib (for serial com on any pin)
#include <TimeLib.h>                            // Time lib Arduino
#include <cstdio>
#include <iostream>                            
#include <TFT_eSPI.h>                           // TFT lib for control of OLED screen
#include <SPI.h>                                // SPI lib. TFT lib depends on this
#include <Wire.h>
#include <EEPROM.h>
#include <JC_Button.h>

// Include files with dependencies
#include "PurpaceMadeLib/sensors_ext/sensors_ext.h"         // File containging classes related to peripheral sensors (GPS, temp, hum, press)
#include "PurpaceMadeLib/ClusterCom/ClusterCom.h"           // ClusterCom library for peer to peer communicatision
#include "PurpaceMAdeLib/DisplayTTGO/DisplayTTGO.h"
                                        


#define DEBUG


volatile long buttonTimer = 0;                   // Variabel som lagrer tiden siden sist knappetrykk
int sleepTimer = TIME_TO_SLEEP * mS_TO_S_FACTOR; // Tid i millisekunder til ESPen går i deep sleep modus

hw_timer_t *hw_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long millis_prev = 0;
bool millisRollover = false;

/**************************************** 
* Instanciate objects
****************************************/

/**************************************** 
* Instanciate objects
****************************************/

// TFT class for OLED display control
TFT_eSPI tft = TFT_eSPI(135, 240);

// GPS class - Constructor takes GPS module baud rate and pins for PS, Tx and Rx
GPS gps(GPSBaud, serialBaud, psGPS, txGPS, rxGPS);

// Weather station class (for simple reading of weather station data)
WeatherStation ws(i2cBME280Adr);

// Ubidots object
Ubidots ubidots(UBIDOTS_TOKEN);

// ClusterCom object
ClusterCom CCom(rxRF, txRF, psRFRX, psRFTX, serialBaud);

// TTGO display
DisplayTTGO display(backLight);

// Unit data 
UnitData unit(batteryPin);

// Button S2
Button S2(btnS2, true);




/****************************************
* Global buffer variables 
****************************************/

// Ubidots
String ubidotsId;
String callbackPayload = "";

// ClusterCom
//uint8_t id = 255;         // Device id / Default = 255
uint8_t mt;                 // Buffer for messageType
String  msgStr;             // Buffer for incoming message
float   msgFloat;
uint8_t from;               // ID to sender of the incoming message

uint8_t numbOfSlaves = 0;   // Remember how many slaves this master has addressed

bool master = false;        // True if device is master


/****************************************
* Sleep Mode Functions
****************************************/

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  if (millis() - buttonTimer > sleepTimer)
  {
    esp_deep_sleep_start();
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void ISRbuttonTimer()
{
  buttonTimer = millis();
}

/****************************************
* Auxiliary Functions
****************************************/

void UbisoftCallback(char *topic, byte *payload, unsigned int length)
{
  callbackPayload = "";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    callbackPayload.concat((char)payload[i]);
  }
  // Serial.println(callbackPayload);

  // for (int i = 0; i < length; i++)
  // {
  //   Serial.print((char)payload[i]);
  // }
  Serial.println();
}


// Functionaly the same as the "delay()" function.
// However, instead of just waiing, the unit is put into light sleep to save power
void espDelay(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}


void ubiPubWeather(String id, float t, float h, float p, float lat, float lng, float alt, int batt){

  // Connection
  if (!ubidots.connected()){                  // Reconnect to ubidots if not connected
      ubidots.reconnect();
      Serial.println("Reconnecting to ubi");
  }
  
  // GPS data treatment
  char gpsData[1000] = "";
  // Format latitude and longitude in a string
  sprintf(gpsData, "\"lat\":%.6f, \"lng\":%.6f", lat, lng);

  // Publish
  // Check if data available                  "Tag"                 Data    Context
  if (t   != 0.0)                 ubidots.add("Temperature",        t               );  // Generate / update temperature variable in Ubidots
  if (h   != 0.0)                 ubidots.add("Humidity",           h               );  // Generate / update humidity variable in Ubidots
  if (p   != 0.0)                 ubidots.add("Pressure [hPa]",     p               );  // Generate / update pressure variable in Ubidots
  ubidots.publish(id.c_str());                       // Publish buffer to Ubidots
  ubidots.loop();

  if (batt!= 0.0)                 ubidots.add("Battery",            batt            );  // Generate / update altitude variable in Ubidots
  if (alt != 0.0)                 ubidots.add("Altitude",           alt             );  // Generate / update altitude variable in Ubidots
  if ((lat!= 0.0) && (lng != 0))  ubidots.add("gps",                1,      gpsData );  // Generate / update GPS variable in Ubidots

  ubidots.publish(id.c_str());                       // Publish buffer to Ubidots
  ubidots.loop();

  // ubidots.subscribe("/v2.0/devices/3/temperature/lv");
  // ubiPubTS = millis();
}




void auxLoop()
{
  gps.refresh(true);                            // Update data from GPS and syncronize time and date
  unit.refresh();                               // Update data from unit (battery state)
  
  // Move relevant data to display object and draw lock screen
  display.setBatteryState(  unit.getBatteryPercent(), 
                            unit.getBatteryVoltage());
  display.setLockScreenData(ws.getTempC(),      // Temperature
                            ws.getHum(),        // Humidity
                            ws.getPressHPa(),   // Pressure
                            gps.getLatitude(),  // Latitude
                            gps.getLongitude(), // Longitude
                            gps.getAltitude()); // Altitude
  display.selectLockScreen();
  display.refresh();


}


void commLoop(){

  if (master){
    if (ubidots.connected() && (millisRollover || ((millis() - ubiPubTS) > ubiPubFreq))){
      ubiPubWeather(  ubidotsId,
                      ws.getTempC(),
                      ws.getHum(),
                      ws.getPressHPa(),
                      gps.getLatitude(),
                      gps.getLongitude(),
                      gps.getAltitude(),
                      unit.getBatteryPercent());
      ubiPubTS = millis();
      Serial.println("Data published to Ubidots");
    }
  }

  // Recive incoming messages from RF module
  if(CCom.available(&mt, &msgStr, &msgFloat, &from))
  {
    switch (mt)
    {
      case CCom.PING:
        CCom.send(ubidotsId.c_str(), from, CCom.PING);    // Respond to ping with device mac address
        break;
      case CCom.ID:   // Respond on slave id request
        if(master)
        {
          if(numbOfSlaves < ALLOWED_SLAVES && from == 0)
          {
            if(CCom.send(msgStr.c_str(), 0, CCom.ID, numbOfSlaves +2));                           // Address between 2-11
            {
              EEPROM.writeString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC * numbOfSlaves, msgStr);   // Store slave device mac address
              EEPROM.write(numbOfSlaves++, NUMB_OF_SLAVES_ADDRESS);                               // Store number of slaves addressed
              //EEPROM.commit();
            }
          }else{
            if(from == 0) CCom.send("Full", 0, CCom.ID);
          }
        }
        break;
      case CCom.SLEEP:    // Go to sleep and for how long
         
        break;
      case CCom.ERROR:    // Handel error that har occured on slave devices
                          // develop in next version   
        break;
      case CCom.DATA:     // Handel data from slave device 
          
          if(msgStr)   Serial.println(msgStr);
          if(msgFloat) Serial.println(msgFloat);

          if(from > 1 && from < ALLOWED_SLAVES+2){   // 1 < from < allowedSlaves, is a valid slave id
            //Push data
            String currentSlaveMac = EEPROM.readString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC *(from -2));

            //ubiPubWeather(currentSlaveMac, );

          }
        break;
      default:            // Unknown massege type
        CCom.send("Unsupported mt", from, CCom.ERROR);
    }

    mt       = 0;
    msgStr.clear();
    msgFloat = 0;
    from     = 255;
  }

  if(!master) // && dataIsReady)   // Push data to master
  {
    ws.getTempC();
    ws.getHum();
    ws.getPressHPa();
    gps.getLatitude();
    gps.getLongitude();
    gps.getAltitude();
    unit.getBatteryPercent();

    //CCom.send();
  }

}




/****************************************
 * Main Functions
****************************************/







//////////////////////////////////////////////////////
//                                                  //
//                       SETUP                      //
//                                                  //
//////////////////////////////////////////////////////
void setup() {
  
  // Start serial communication with microcontroller
  Serial.begin(serialBaud);  
  espDelay(1000);

  EEPROM.begin(EEPROM_SIZE);

  display.selectMessageScreen();
  display.setString(0, "Startup");
  display.setString(2, "Unit is booting up.");
  display.setString(3, "Please wait...");
  display.setString(5, "Enabling peripheral");
  display.setString(6, "units...");
  display.refresh();  

  // Button object begin to enable monitoring of 
  S2.begin();

  // Enable external sensors
  gps.enable();                                 // Power up GPS module and establish serial com 
  ws.enable();                                  // Establish I2C com with Weather Station
  CCom.enable();                                // Power up radio transmission modules
  unit.enable();                                // Set pinmode for battery surveilance
  espDelay(1000); 
  unit.refresh();

  ubidotsId = WiFi.macAddress();
  
  // MASTER / SLAVE
  display.setString(0, "Startup");
  display.setString(2, "For \"Master\"");
  display.setString(3, "configuration:");
  display.setString(5, "Press right button..");
  unsigned long selTS = millis();
  int selTime = 5000;
  do{
    S2.read(); // Read
    if (S2.isPressed()){
      master = true;
    }
    // Display remaining time before configured as slave
    display.setString(6, String(selTime/1000 - ((millis() - selTS) / 1000)));
    display.refresh();
  } while ((millis() < selTS + selTime) && (!S2.isPressed()));

  display.setString(0, "Startup");
  display.setString(2, "Unit configured");
  display.setString(5, "");
  display.setString(6, "");




  //////////////////////////////////////////////////////
  //                                                  //
  //                       MASTER                     //
  //                                                  //
  //////////////////////////////////////////////////////

  if (master){
    display.setString(3, "as \"Master\"!");
    display.setMaster();                          // Display master symbol besides clock

    // UBIDOTS
    display.setString(5, "Connecting to");
    display.setString(6, "Ubidots...");
    display.refresh();                        
    // ubidots.setDebug(true);  // uncomment this to make debug messages available
    ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
    ubidots.setCallback(UbisoftCallback);
    ubidots.setup();
    ubidots.reconnect();
    ubiPubTS = millis(); 
    display.setString(5, "Connected!");
    display.setString(6, "");
    display.refresh();   
    espDelay(1000);

    // CLUSTERCOM
    // Setup crypt key and eeprom storage
    display.setString(5, "Configuring ");
    display.setString(6, "ClusterCom...");
    display.refresh();
    CCom.begin(UBIDOTS_TOKEN, EEPROM_SIZE, ID_EEPROME_ADDRESS);
    CCom.setId(CCom.masterId, true);                        // Set device to master id
    if(EEPROM.read(NUMB_OF_SLAVES_ADDRESS) != 255) 
      numbOfSlaves = EEPROM.read(NUMB_OF_SLAVES_ADDRESS);   // Recall from storage

    display.setString(5, "Configured!");
    display.setString(6, "");
    display.refresh();
    espDelay(1000);
  }



  //////////////////////////////////////////////////////
  //                                                  //
  //                       SLAVE                      //
  //                                                  //
  //////////////////////////////////////////////////////

  else{
    display.setString(3, "as \"Slave\"");

    // CLUSTERCOM
    // Setup crypt key and eeprom storage
    display.setString(5, "Configuring ");
    display.setString(6, "ClusterCom...");
    display.refresh();
    CCom.begin(UBIDOTS_TOKEN, EEPROM_SIZE, ID_EEPROME_ADDRESS);

    bool err = !CCom.getId();
    if (err){
      Serial.println("Failed to obtain id");
      display.setString(5, "Failed to");
      display.setString(6, "obtain ID");
      display.refresh();
      espDelay(1000);
    }

    else{
      Serial.println("ID obtained");
      display.setString(5, "ID obtained");
      display.setString(6, "");
      display.refresh();
      espDelay(1000);
    }
  }



  // Sleep configuration
  hw_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(hw_timer, &onTimer, true);
  timerAlarmWrite(hw_timer, 1000000, true);
  timerAlarmEnable(hw_timer);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  attachInterrupt(btnS1, ISRbuttonTimer, RISING);
  attachInterrupt(btnS2, ISRbuttonTimer, RISING);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);

  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup:Button");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup:Timer");
    break;
  }

}

void loop(){
  
  // millis() rollover check
  if(millis() < millis_prev) millisRollover = true;
  else millisRollover = false;

  // Draw Ubidots connection symbol
  if (master){
    ubidots.loop();
    if (ubidots.connected()) display.setUbi();
    else {display.resetUbi(); ubidots.reconnect();}
    display.refresh();
  }

  // Loops
  auxLoop();                                    // Loop all auxiliary utensils
  commLoop();                                   // Loop communication utensils
  
  millis_prev = millis();

  delay(2000);
  
}