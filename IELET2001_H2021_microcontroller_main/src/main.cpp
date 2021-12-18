
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
#define TEST

volatile unsigned long   buttonTS = 0;                              // Variabel som lagrer tiden siden sist knappetrykk
volatile bool   buttonPress;                                  // Woken by button
bool            timerWakeup = false;
int             awakeTime = TIME_TO_SLEEP_CHECK * mS_TO_S_FACTOR;      // Awake time after button push

esp_sleep_wakeup_cause_t wakeup_reason;
hw_timer_t *hw_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long millis_prev = 0;
bool          millisRollover = false;
int           hour_prev = 0;
bool          hourChange = false;


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
bool masterDataPublished;

// Sensors_ext
bool dataIsReady = false;

// Array containging sensor data for unit
// Temp[0] , Hum[1] , Press[2] , Lat[3] , Lng[4] , Alt[5] , Batterypercent[6] , BatteryVolt[7]
float sensorData[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// ClusterCom
//uint8_t id = 255;         // Device id / Default = 255
uint8_t mt;                 // Buffer for messageType
String  msgStr;             // Buffer for incoming message
float   msgFloat;
uint8_t from;               // ID to sender of the incoming message
bool    CComErr;

uint8_t numbOfSlaves = 0;   // Remember how many slaves this master has addressed
String  macSlave[ALLOWED_SLAVES];

bool master = false;        // True if device is master
unsigned long setupTime;    // Get time after setup

bool retryFetchSlaveData = true;  // Master is trying to fetch date from connected slaves


/****************************************
* Sleep Mode Functions
****************************************/
/*
void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  
  if ((millis() - buttonTimer > awakeTime))
  {
    esp_deep_sleep_start();
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}
*/

void IRAM_ATTR ISRbuttonTimer()
{
  buttonTS = millis();
  buttonPress = true;
}

// Functionaly the same as the "delay()" function.
// However, instead of just waiting, the unit is put into light sleep to save power
void espDelay(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

// Wakes unit up every whole hour
unsigned long calcSleepTimeSec(){
  return ((59-minute()) * 60) + (60-second());
}

// Puts unit in deep sleep and sets wakeup timer to next full hour
void goToDeepSleep(){
  esp_sleep_enable_timer_wakeup(calcSleepTimeSec() * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}
/****************************************
* UBIDOTS
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

void ubiPubWeather(String id, float t, float h, float p, float lat, float lng, float alt, int batt){

  // Connection
  if (!ubidots.connected()){                  // Reconnect to ubidots if not connected
      ubidots.reconnect();
      ubidots.loop();
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
  Serial.print("Unit ID data pub UBI: ");
  Serial.println(id);
  ubidots.loop();

  // ubidots.subscribe("/v2.0/devices/3/temperature/lv");
  // ubiPubTS = millis();
}

bool ubiPubData(String ID, float* Data){
// Publish data fethed from slave unit
  if (ubidots.connected() && (millisRollover || ((millis() - ubiPubTS) > ubiPubFreq))){
    ubiPubWeather(  ID,
                    Data[TEMP    ],  // Temp
                    Data[HUM     ],  // Hum
                    Data[PRESS   ],  // Press
                    Data[LAT     ],  // Latitude
                    Data[LNG     ],  // Longitude
                    Data[ALT     ],  // Altitude
                    Data[BATPERC ]); // Battery Percent
    ubiPubTS = millis();
    Serial.println("Data published to ubi");
    Serial.println(Data[BATPERC]);
    return true;
  }
  return false;
}


//////////////////////////////////////////////////////
//                                                  //
//                 GROUP FUNCTIONS                  //
//                                                  //
//////////////////////////////////////////////////////

/****************************************
 * AUX
****************************************/

// Gathers all necessarry data and returns "true" if data is gathered successfully
bool dataFetchLoop(bool getGPS){
  
  bool _dataOK[8] = {false, false, false, false, false, false, false, false};
  
  if (getGPS){
    gps.refresh(true);                            // Update data from GPS and syncronize time and date 
    sensorData[LAT]     = gps.getLatitude();      // Latitude
    sensorData[LNG]     = gps.getLongitude();     // Longitude
    sensorData[ALT]     = gps.getAltitude();      // Altitude
  }
  unit.refresh();                                 // Update data from unit (battery state)
  sensorData[TEMP]    = ws.getTempC();            // Temperature
  sensorData[HUM]     = ws.getHum();              // Humidity
  sensorData[PRESS]   = ws.getPressHPa();         // Pressure
  sensorData[BATPERC] = unit.getBatteryPercent(); // Battery Percent
  sensorData[BATVOLT] = unit.getBatteryVoltage(); // Battery Voltage

  // Do not include temperature data == 0.0 in comparison, since it's likely that it can be 0
  for (int i=1; i<8 ; i++){
    if (sensorData[i] != 0.0) _dataOK[i] = true;
  }

  // Compensate if gps data is not activated
  if (!getGPS){
    for (int i=3; i<6; i++){_dataOK[i] = true;}
  }

  // Sum up if all data have been retrieved successfully
  bool _ok = true; 
  for (int i=1; i<8 ; i++){
    if (!_dataOK[i]) _ok = false;
  }

  // Return sum of all data gathering
  return _ok;
}

void displayLoop(){
  // Move relevant data to display object and draw lock screen
  display.setLockScreenData(sensorData[TEMP],     // Temperature
                            sensorData[HUM],      // Humidity
                            sensorData[PRESS],    // Pressure
                            sensorData[LAT],      // Latitude
                            sensorData[LNG],      // Longitude
                            sensorData[ALT]);     // Altitude
  display.setBatteryState(  sensorData[BATPERC],  // Battery Percent
                            sensorData[BATVOLT]); // Battery Voltage
  
  
  if (master){
    // Check if connection to Ubidots is valid, and show in display 
    if (ubidots.connected()) display.setUbi();
    else display.resetUbi();
  }

  // Select right screen config and refresh to push changes   
  display.selectLockScreen();
  display.refresh();
}


/****************************************
 * COMMUNICATION
****************************************/

bool getSlaveData(uint8_t slaveUnit, float* slaveData){

  bool     err  = false;
  uint16_t wait = 1000;  // 1sec
  uint8_t  i    = 0;

  String dataName[] = {"temp", "hum", "press", "lat", "lng", "alt", "batperc"};
  
  // -- Get slave data
  for(String name : dataName)
  {
    delay(50);
    err = !CCom.send(name.c_str(), slaveUnit+2, CCom.DATA);    // Send and wait for response for a desired amount of time
    if(!err) err = !CCom.available(&mt, &msgStr, &msgFloat, &from, wait);
    if(err) {err = true; break;} 
    
    if(mt == CCom.DATA && from == slaveUnit+2)    // Check desired data is resived and store to array
      slaveData[i++] = msgFloat;

    if(mt != CCom.ERROR) continue;                // Continue if no error
    
    err = true;
    Serial.print("ERROR");
    Serial.println(msgStr);
  }

  if(!err) CCom.send("Sleep", slaveUnit+2, CCom.SLEEP);    // Tell slave to speep

  // Clear buffers
  mt       = 0;
  msgStr.clear();
  msgFloat = 0;
  from     = 255;

  return err ? false : true;
}


void commLoop(){

  // Recive incoming messages from RF module
  if(CCom.available(&mt, &msgStr, &msgFloat, &from))
  {
    //Serial.println(mt);
    //if(msgStr)   Serial.println(msgStr);
    //if(msgFloat) Serial.println(msgFloat);
    //Serial.println(from);

    switch (mt)
    {
      case CCom.PING:
        
        break;


      case CCom.ID:   // Respond on id request

        if(master)
        {
          if(numbOfSlaves < ALLOWED_SLAVES && from == 0)
          {
            if(CCom.send(msgStr.c_str(), 0, CCom.ID, ++numbOfSlaves +1))    // Address from 2
            {
              macSlave[numbOfSlaves-1] = msgStr;
              EEPROM.writeString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC * numbOfSlaves-1, msgStr);   // Store slave device mac address
              EEPROM.write(numbOfSlaves, NUMB_OF_SLAVES_ADDRESS);                                   // Store number of slaves addressed
              //EEPROM.commit();
            }
            else numbOfSlaves--;
          }

          else if(from == 0) 
          {
            uint8_t i = 0;
            while(macSlave[i++] != "FF:FF:FF:FF:FF:FF")   // Check if device is deleted
              if(i >= ALLOWED_SLAVES) break;     
            if(i < ALLOWED_SLAVES) 
            {
              if(CCom.send(msgStr.c_str(), 0, CCom.ID, i+1))                            // Address from 2
              {
                EEPROM.writeString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC * (i-1), msgStr);   // Store slave device mac address
                //EEPROM.commit();
              }
            }
            else CCom.send("full", 0, CCom.ID);
          }

          else{
            if(msgStr == "delete") 
            { 
              EEPROM.writeString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC *(from -2), "FF:FF:FF:FF:FF:FF");   // Delete device
              //EEPROM.commit();
            }
          }
        }

        else CCom.send(ubidotsId.c_str(), from, CCom.ID);    // Respond to id with device mac address
        break;


      case CCom.SLEEP:                                       // Go to sleep

        // Set timer to wake unit next full hour
        goToDeepSleep();
        break;


      case CCom.ERROR:    // Handle error that har occured on slave devices
                                        
        Serial.print("ERROR: ");
        Serial.println(msgStr);                              // develop action in next version   
        break;


      case CCom.DATA:     // Handle data from device 

        if(master)        // Data from slave
        {
          if(msgStr)   Serial.println(msgStr);
          if(msgFloat) Serial.println(msgFloat);

          if(from > 1 && from < ALLOWED_SLAVES+2)    // 1 < from < allowedSlaves, is a valid slave id
          {
            String currentSlaveMac = macSlave[from -2];
            // Handle data...
          }
        }

        else{              // Data from master
          if(dataIsReady)
          {
                 if(msgStr == "temp"   ) CCom.send(sensorData[TEMP   ], from, CCom.DATA);
            else if(msgStr == "hum"    ) CCom.send(sensorData[HUM    ], from, CCom.DATA);
            else if(msgStr == "press"  ) CCom.send(sensorData[PRESS  ], from, CCom.DATA);
            else if(msgStr == "lat"    ) CCom.send(sensorData[LAT    ], from, CCom.DATA);
            else if(msgStr == "lng"    ) CCom.send(sensorData[LNG    ], from, CCom.DATA);
            else if(msgStr == "alt"    ) CCom.send(sensorData[ALT    ], from, CCom.DATA);
            else if(msgStr == "batperc") CCom.send(sensorData[BATPERC], from, CCom.DATA);
            else  CCom.send("Doesn't support this data", from, CCom.ERROR);
          }
          else CCom.send("Data not ready", from, CCom.ERROR);
        }          
        break;


      default:            // Unknown massege type
        //CCom.send("Device dosn't support mt", from, CCom.ERROR);
        break;
    }

    mt       = 0;
    msgStr.clear();
    msgFloat = 0;
    from     = 255;
  }

  if(!master && dataIsReady && 0)   // Push data to master, !!function is delayed to future development (&& 0)
  {
    // Get data

    //CCom.send();

    // goToDeepSleep() ??
  }

}


//////////////////////////////////////////////////////
//                                                  //
//                    MAIN LOOPS                    //
//                                                  //
//////////////////////////////////////////////////////


/****************************************
 * SETUP
****************************************/
void setup() {
  delay(1000);

  EEPROM.begin(EEPROM_SIZE);

  #ifdef WIPE_EEPROM
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 255);
    }
    EEPROM.commit();
    delay(500); 
  #endif
  
  // Enable hardware timer for wakeup
  /*
  hw_timer = timerBegin(3, 80, true);
  timerAttachInterrupt(hw_timer, &onTimer, true);
  timerAlarmWrite(hw_timer, 1000000, true);
  timerAlarmEnable(hw_timer);
  */

  // Enable wakeup button
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);

  // Attach interrupt to button
  attachInterrupt(btnS2, ISRbuttonTimer, FALLING);

  // Reason for wakeup
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason){

    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup:Button");
      timerWakeup = false;
      buttonPress = true;
      buttonTS = millis();
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup:Timer");
      timerWakeup = true;
      buttonPress = false;
      break;
  }

  // Select if screen should be enabled based on how it is woken
  if (timerWakeup)  display.disable();
  else              display.enable();

  // Start serial communication with microcontroller
  Serial.begin(serialBaud);  
  delay(1000);

  

  // Button object begin to enable monitoring of 
  S2.begin();

  // Startup sequence
  display.selectMessageScreen();
  display.setString(0, "Startup");
  Serial.println("Startup");
  delay(1000);

  // Decide if new configuration is needed, and what actions to take if allready configured
  Serial.println(EEPROM.read(CONFIG_STATE));
  switch (EEPROM.read(CONFIG_STATE)){
    
    // First boot-up. Configuration is needed
    case NOT:{
      
      Serial.println("Not conf.");
      // Master / Slave configuration
      display.setString(2, "For \"Master\"");
      display.setString(3, "configuration:");
      display.setString(5, "Press right button..");
      unsigned long selTS = millis();
      do{
        S2.read(); // Read
        if (S2.isPressed()){
          master = true;
        }
        // Display remaining time before configured as slave
        display.setString(6, String(selTime/1000 - ((millis() - selTS) / 1000)));
        display.refresh();
      } while ((millis() < selTS + selTime) && (!S2.isPressed()));
      
      if (master) EEPROM.write(CONFIG_STATE, MASTER);
      else        EEPROM.write(CONFIG_STATE, SLAVE);
      // EEPROM.commit();

      break;}
    
    
    // Configuration is allready performed 
    case MASTER:{
      Serial.println("Master");
      master = true;
      break;}
    
    // Configuration is allready performed  
    case SLAVE:{
      Serial.println("Slave");
      master = false;
      break;}
  }
  
  display.setString(2, "Unit is booting up.");
  display.setString(3, "Please wait...");
  display.setString(5, "Enabling peripherals");
  display.setString(6, "");
  display.refresh();
  
  
  // Enable external sensors                              

  if (hour() == 12) gps.enable();               // Only enable GPS module if time is current hour is 12
  ws.enable();                                  // Establish I2C com with Weather Station
  CCom.enable();                                // Power up radio transmission modules
  unit.enable();                                // Set pinmode for battery surveilance
  delay(1000); 
  unit.refresh();                               // Read battery data
  display.setBatteryState(unit.getBatteryPercent(), unit.getBatteryVoltage());

  ubidotsId = WiFi.macAddress();                // Read MAC to use as unit name in Ubidots

  display.setString(0, "Startup");
  display.setString(2, "Unit configured");
  display.setString(5, "");

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
    delay(1000);

    // CLUSTERCOM
    // Setup crypt key and eeprom storage
    display.setString(5, "Configuring ");
    display.setString(6, "ClusterCom...");
    display.refresh();
    CCom.begin(UBIDOTS_TOKEN, EEPROM_SIZE, ID_EEPROME_ADDRESS);
    CCom.setId(CCom.masterId, true);                        // Set device to master id
    if(EEPROM.read(NUMB_OF_SLAVES_ADDRESS) != 255) 
      numbOfSlaves = EEPROM.read(NUMB_OF_SLAVES_ADDRESS);   // Recall from storage
    for(int slaveUnit=0; slaveUnit < numbOfSlaves; slaveUnit++)
    {
      macSlave[slaveUnit] = EEPROM.readString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC*slaveUnit);
    }

    display.setString(5, "Configured!");
    display.setString(6, "");
    display.refresh();
    delay(1000);

    masterDataPublished = false;
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
    delay(100);

    CComErr = !CCom.getId();

    if (CComErr){
      Serial.println("Failed to obtain id");
      display.setString(5, "Failed to");
      display.setString(6, "obtain ID");
      display.refresh();
      delay(1000);
    }

    else{
      Serial.println("ID obtained");
      display.setString(5, "ID obtained");
      display.setString(6, "");
      display.refresh();
      delay(1000);
    }
  }

  setupTime = millis();
}





/****************************************
 * MAIN
****************************************/
void loop(){
  
  if (buttonPress) display.enable();

  // millis() rollover check
  if(millis() < millis_prev)  millisRollover = true;
  else                        millisRollover = false;

  // hour() change check. Do not set false if not, because it will be an indicator that an hour change has been ,ade during the process and that data shoiuld be transmitted
  if(hour() != hour_prev) hourChange = true;

  // Update data from unit
  dataIsReady = dataFetchLoop(false && hour()==12);    // Get gps data only when the hour says 12. Returns true when requested data is updated
  
  // Update display
  displayLoop();

  /* 
  // If year is not updated, do not continue until GPS has updated time and date (To be able so sync units)
  if (year()<= 2020){
    int scan = 0;
    gps.enable();  
    while (year()<= 2020){

      // Startup sequence
      display.selectMessageScreen();
      display.setString(0, "GPS");
      display.setString(2, "Synchronizing");
      display.setString(3, "time and date");
      display.setString(5, "Please wait");
      
      // Loading symbol
      if      (scan == 0) display.setString(6, "|");
      else if (scan == 1) display.setString(6, "/");
      else if (scan == 2) display.setString(6, "-");
      else if (scan == 3) display.setString(6, "\\");
      
      gps.refresh(true);
      unit.refresh();
      display.refresh();

      if (scan < 3) scan ++;
      else scan = 0;

      delay(100);

    }
  }
  */

  /*******************************************************
  * Communicate data between units and publish to Ubidots
  ********************************************************/
  
  // Is triggered either by periodical wakeuptimer or by an hour change if wakeup was triggered by button
  if (true || timerWakeup || hourChange){
    // Start gathering data and publish to ubidots
 
    ////////////////////
    //     MASTER     //
    ////////////////////

    // Master communication behaviour
    if(master){
      
      // UBIDOTS - Check if connected and reconnect if not
      ubidots.loop();
      if (!ubidots.connected()) ubidots.reconnect();

      // Start communication Listen for incoming data
      // Scan local area for new slaves
      commLoop();

      // Publish Master data if not allready done. Make 5 attempts with 1 second spacing
      if (!masterDataPublished && dataIsReady){
        for (int i = 0; i<5; i++){
          if (masterDataPublished) break;
          masterDataPublished = ubiPubData(ubidotsId, sensorData);
          delay(1000);
        }
      } 

      // Fetch data from slaves 
      uint16_t timeout = 10000; //10sec

      if((millis() - setupTime) > timeout && numbOfSlaves && retryFetchSlaveData){ 
      // Don't fetch data right after boot,
      // and check if any slaves are set up

        /* // Fetch data from slave units
          -- Request data from the given slave number
          -- Wait for data from the given slave
          -- If no data recieved, stash unit number in an array, 
          -- to request again at the end of the original list
          -- Push data to the cloud
        */
      
        bool resivedData[numbOfSlaves] = {1};
        for (int slaveUnit = 0; slaveUnit < numbOfSlaves; slaveUnit++){

          String slaveID = macSlave[slaveUnit];
          if(slaveID == "FF:FF:FF:FF:FF:FF") continue;    // Skip unit if it is deleted

          // Temp[0] Hum[1] Press[2] Lat[3] Lng[4] Alt[5] BatteryPercent[6]
          float slaveData[7];
        
          // If error, store id for a second attempt
          if(!getSlaveData(slaveUnit, slaveData)) { resivedData[slaveUnit] = false; continue; }  

          unsigned long TS = millis();
          while(!ubiPubData(slaveID, slaveData))
          {
            if((millis()-TS) > ubiPubFreq*2)
            {
              Serial.println("ERROR: can not publish to Ubidots");
              break;
            } 
          }
        }


        // Do a secound attempt to get data from slave units
        uint8_t slaveUnit = 0;

        for(bool state : resivedData)
        {
          slaveUnit++;
          if(state) continue;   // Move on to next devise

          String slaveID = macSlave[slaveUnit];

          // Temp[0] Hum[1] Press[2] Lat[3] Lng[4] Alt[5] BatteryPercent[6]
          float slaveData[7];

          if(!getSlaveData(slaveUnit, slaveData)) continue;

          unsigned long TS = millis();
          while(!ubiPubData(slaveID, slaveData))
          {
            if((millis()-TS) > ubiPubFreq*2)
            { 
              Serial.println("ERROR: can not publish to Ubidots"); 
              break; 
            } 
          }
          resivedData[slaveUnit] = true;
        }
        retryFetchSlaveData = false;
      }
    }



    ////////////////////
    //      SLAVE     //
    ////////////////////

    else {

      // Attempt to get ID form master if none is given
      if (!master && CComErr) CComErr = !CCom.getId(); 

      // Communication with master unit
      commLoop();

    }
  }


  /////////////////////
  //  END PROCEDURE  //
  /////////////////////

  // Decide if unit shoud be put into deep sleep or only shut off display
  
  // Only valid if the buttonpush triggered the wakeup
  if      (!hourChange && !timerWakeup && (millis() - buttonTS > awakeTime))  goToDeepSleep();    // Go to deep sleep if no other activity
  else if (millis() - buttonTS > awakeTime){                                  display.disable(); buttonPress = false;}  // Only turn off display if timer runs out but other activity
  
  // Go to deep sleep if unit has attempted to get data from other units for x amount of seconds
  if ((millis() - setupTime > timeDataFetchSlaves) || millisRollover)         goToDeepSleep();


  millis_prev = millis();
  hour_prev = hour();
  delay(0);                                  // Scan frequency
  
}


