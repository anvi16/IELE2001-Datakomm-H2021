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
#include "config.h"                                         // Config file for HW pins, baud etc.

#define DEBUG


volatile long buttonTimer = 0;                   // Variabel som lagrer tiden siden sist knappetrykk
int sleepTimer = TIME_TO_SLEEP * mS_TO_S_FACTOR; // Tid i millisekunder til ESPen går i deep sleep modus

hw_timer_t *hw_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;




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
* Variables Functions
****************************************/

String callbackPayload = "";
bool firstScan = HIGH;

// ClusterCom
uint8_t id = 255;      // Device id / Default = 255
uint8_t numbOfSlaves = 0;
String msg;          // Buffer for incoming message
uint8_t mt;          // Buffer for messageType
bool master = false; // True if device is master


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


void auxLoop()
{
  gps.refresh(true);                            // Update data from GPS and syncronize time and date
  unit.refresh();                               // Update data from unit (battery state)
  ws.refresh();
  
  // Move relevant data to display object and draw lock screen
  display.setBatteryState(  unit.getBatteryPercent(), 
                            unit.getBatteryVoltage());
  display.setLockScreenData(ws.getTempC(), 
                            ws.getHum(), 
                            ws.getPressHPa(), 
                            gps.getLatitude(),
                            gps.getLongitude(),
                            gps.getAltitude());
  display.selectLockScreen();
  display.refresh();

  espDelay(3000);

  //display.selectBlackScreen();
  //display.refresh();

  //espDelay(3000);

}


void commLoop()
{
/*
  WeatherData weatherData; 

  for (int unit = 0; unit < 100; unit++){
    weatherData.lat = 0;
    weatherData.lng = 0;
    weatherData.alt = 0;

  }

  

  // Format latitude and longitude in a string
  char gpsData[1000] = "";
  sprintf(gpsData, "\"lat\":%.6f, \"lng\":%.6f", weatherData.lat, weatherData.lng);

*/  




  // Ubidots
  if (!ubidots.connected()){                    // Reconnect to ubidots 
    ubidots.reconnect();
    ubidots.subscribe("/v2.0/devices/3/temperature/lv");
  }

  // triggers the routine every 5 seconds
  if (abs(millis() - ubiPubTS) > ubiPubFreq) 
  {
    char gpsData[1000] = "";
    
    // Format latitude and longitude in a string
    sprintf(gpsData, "\"lat\":%.6f, \"lng\":%.6f", gps.getLatitude(), gps.getLongitude());

    // Ubidots publish
    //          "Variable label"  "Value"         "Context"
    ubidots.add("gps",            1,              gpsData   );  // Generate / update GPS variable in Ubidots 
    ubidots.add("temperature",    ws.getTempC()             );  // Generate / update temperature variable in Ubidots
    ubidots.add("Humidity",       ws.getHum()               );  // Generate / update humidity variable in Ubidots
    ubidots.add("Pressure [hPa]", ws.getPressHPa()          );  // Generate / update pressure variable in Ubidots

    // ubidots.publish(DEVICE_LABEL);              // Publish buffer to Ubidots
    ubidots.publish("3");              // Publish buffer to Ubidots

    // Serial.println();
    // Serial.println("SUB: ");
    // Serial.println(ubidots.subscribe("/v2.0/devices/3/temperature/lv"));
    // Serial.println();

    ubidots.subscribe("/v2.0/devices/3/temperature/lv");

    ubiPubTS = millis();
  }
  ubidots.loop();

  // Recive incomming messages from RF module
  if(CCom.available(mt, msg))
  {
    Serial.println(mt);
    Serial.println(msg);

    
    switch (mt)
    {
      case CCom.PING:
        /* code */
        break;
      case CCom.ID:
        if(master)
        {
          if(numbOfSlaves < 100)
          {
            CCom.send(msg.c_str(), 255, CCom.ID, numbOfSlaves +2); // Address between 2-101 
            EEPROM.write(numbOfSlaves++, NUMB_OF_SLAVES_ADDRESS);
            //EEPROM.commit();
          }

        }
        break;
      case CCom.SLEEP:
        /* code */
        break;
      case CCom.ERROR:
        /* code */
        break;
      case CCom.DATA:
        /* code */
        break;
      default:
          // Unknown massege type
        break;
    }
  }

}


void serialPrints()
{
  Serial.println();
  Serial.print("MQTT payload size: ");
  Serial.println(sizeof(callbackPayload));
  Serial.println();
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
    CCom.setId(1, true);                        // Set device to master id

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
 
  auxLoop();                                    // Loop all auxiliary utensils
  //commLoop();                                   // Loop communication utensils
  //serialPrints();                               // Run all desired prints
  
}