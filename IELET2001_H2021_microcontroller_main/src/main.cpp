
#include "config.h"                                 // Config file for HW pins, baud etc.

// Include libraries
#include <Arduino.h>                                // General Arduino C++ lib
#include <TinyGPS++.h>                              // GPS lib for easy extraction of GPS data
#include <SoftwareSerial.h>                         // Software serial lib (for serial com on any pin)
#include <TimeLib.h>                                // Time lib Arduino
#include <cstdio>
#include <iostream>                            
#include <TFT_eSPI.h>                               // TFT lib for control of OLED screen
#include <SPI.h>                                    // SPI lib. TFT lib depends on this
#include <Wire.h>
#include <EEPROM.h>
#include <JC_Button.h>

// Include files with dependencies
#include "PurpaceMadeLib/sensors_ext/sensors_ext.h" // File containging classes related to peripheral sensors (GPS, temp, hum, press)
#include "PurpaceMadeLib/ClusterCom/ClusterCom.h"   // ClusterCom library for peer to peer communicatision
#include "PurpaceMAdeLib/DisplayTTGO/DisplayTTGO.h"
                                        
#define DEBUG
#define TEST

volatile unsigned long   buttonTS = 0;               // Variabel som lagrer tiden siden sist knappetrykk
volatile bool   buttonPress;                         // Woken by button
bool            timerWakeup = false;
int             awakeTime = TIME_TO_SLEEP_CHECK * mS_TO_S_FACTOR; // Awake time after button push

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
//uint8_t id = 255;                               // Device id / Default = 255
uint8_t mt;                                       // Buffer for messageType
String  msgStr;                                   // Buffer for incoming message
float   msgFloat;
uint8_t from;                                     // ID to sender of the incoming message
bool    CComErr;

uint8_t numbOfSlaves = 0;                         // Remember how many slaves this master has addressed
String  macSlave[ALLOWED_SLAVES];

bool master = false;                              // True if device is master
unsigned long setupTime;                          // Get time after setup

bool retryFetchSlaveData = true;                  // Master is trying to fetch date from connected slaves


/****************************************
* Sleep Mode Functions
****************************************/




/* 
void IRAM_ATTR ISRbuttonTimer()
{
  buttonTS = millis();
  buttonPress = true;
}
 */








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
  
  #ifdef DEBUG
    Serial.println("Data fetch from sensors");
    if (getGPS) Serial.println("GPS included");
  #endif
  
  bool _dataOK[8] = {false, false, false, false, false, false, false, false};
  
  if (getGPS && gps.isEnabled()){
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
  
  #ifdef DEBUG
    Serial.println("Display loop called");
    Serial.print("Temperature: ");
    Serial.print(sensorData[TEMP]);               // Sample variable to be printet to show what is read
  #endif
  
  display.selectLockScreen();
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
    ubidots.loop();                               // Maintain Ubidots connection
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

  if(!err) CCom.send("Sleep", slaveUnit+2, CCom.SLEEP); // Tell slave to speep

  // Clear buffers
  mt       = 0;
  msgStr.clear();
  msgFloat = 0;
  from     = 255;

  return err ? false : true;
}

void commLoop(){

  /* // Recive incoming messages from RF module
   
    -- Check if there is a valid message in buffer
    -- If there is a message check what messageType(mt) it has,
      and compleate desired tasks. 

    -- Switch(mt)
         PING  -  Respond only with ack
         ID    -  Master:
                     Look up available id's and address slaveUnit
                     If payload (msgStr) is "delete" remove slave from memory 
                     and if all addresses is in use respond with msg "Master is full" 
                  Slave:
                     Send back mac address
         SLEEP - Send unit to sleep 
         ERROR - Print error messages
         DATA  - Master:
                   Handel requests from slave
                 Slave:
                   Return data requested in payload (msgStr) if data is ready to be sent,
                   else respont with "Data not ready",
                   and if requested data is not fount send error "Doesn't support this data"

         default - return devise does not support this message type 

    -- Clear local buffers after request is handeld and get ready to resive new request
  */

  if(CCom.available(&mt, &msgStr, &msgFloat, &from))
  {
    switch (mt)
    {
      case CCom.PING:
        
        break;


      case CCom.ID:                               // Respond on id request

        if(master)
        {
          if(numbOfSlaves < ALLOWED_SLAVES && from == 0)
          {
            if(CCom.send(msgStr.c_str(), 0, CCom.ID, ++numbOfSlaves +1))    // Address from 2
            {
              macSlave[numbOfSlaves-1] = msgStr;
              EEPROM.writeString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC * numbOfSlaves-1, msgStr);   // Store slave device mac address
              EEPROM.write(numbOfSlaves, NUMB_OF_SLAVES_ADDRESS);                                   // Store number of slaves addressed
              if(EEPROM_COMMIT) EEPROM.commit();
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
              if(CCom.send(msgStr.c_str(), 0, CCom.ID, i+1))                                 // Address from 2
              {
                EEPROM.writeString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC * (i-1), msgStr);   // Store slave device mac address
                if(EEPROM_COMMIT) EEPROM.commit();
              }
            }
            else CCom.send("Master is full", 0, CCom.ERROR);
          }

          else{
            if(msgStr == "delete") 
            { 
              EEPROM.writeString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC *(from -2), "FF:FF:FF:FF:FF:FF");   // Delete device
              if(EEPROM_COMMIT) EEPROM.commit();
            }
          }
        }

        else CCom.send(ubidotsId.c_str(), from, CCom.ID);  // Respond to id with device mac address
        break;


      case CCom.SLEEP:                            // Go to sleep

        goToDeepSleep();                          // Enter deep sleep. Wakeup calculated to be at next full hour                 
        break;


      case CCom.ERROR:                            // Handle error that har occured on slave devices
                                        
        Serial.print("ERROR: ");
        Serial.println(msgStr);                   // develop action in next version   
        break;


      case CCom.DATA:     // Handle data from device 

        if(master)        // Data from slave
        {
          if(msgStr)   Serial.println(msgStr);
          if(msgFloat) Serial.println(msgFloat);

          if(from > 1 && from < ALLOWED_SLAVES+2) // 1 < from < allowedSlaves, is a valid slave id
          {
            String currentSlaveMac = macSlave[from -2];
            // Handle data...
          }
        }

        else{                                     // Data from master
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


      default:                                    // Unknown massege type
        //CCom.send("Device dosn't support mt", from, CCom.ERROR);
        break;
    }

    // Clear local buffers
    mt       = 0;
    msgStr.clear();
    msgFloat = 0;
    from     = 255;
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
  //attachInterrupt(btnS2, ISRbuttonTimer, FALLING);

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
  if (timerWakeup)  display.disable();            // Turn display off to save power
  else              display.enable();             // Enable display if button is pressed or first boot

  // Start serial communication with microcontroller
  Serial.begin(serialBaud);  
  delay(1000);

  // Button object begin to enable monitoring of 
  pinMode(btnS2, INPUT);
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
        if (S2.read()) {buttonTS = millis(); buttonPress = true;}        // Read
        if (S2.isPressed()){
          master = true;
        }
        // Display remaining time before configured as slave
        display.setString(6, String(selTime/1000 - ((millis() - selTS) / 1000)));
        display.refresh();
      } while ((millis() < selTS + selTime) && (!S2.isPressed()));
      
      if (master) EEPROM.write(CONFIG_STATE, MASTER);
      else        EEPROM.write(CONFIG_STATE, SLAVE);
      if(EEPROM_COMMIT) EEPROM.commit();
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

  if ((hour() == gpsCheckHour1) ||
      (hour() == gpsCheckHour2)||
      (year() < 2020)){
    gps.enable();                                 // Only enable GPS module if needed
  }
  ws.enable();                                    // Establish I2C com with Weather Station
  CCom.enable();                                  // Power up radio transmission modules
  unit.enable();                                  // Set pinmode for battery surveilance
  delay(1000); 
  unit.refresh();                                 // Read battery data
  display.setBatteryState(unit.getBatteryPercent(), unit.getBatteryVoltage());

  ubidotsId = WiFi.macAddress();                  // Read MAC to use as unit name in Ubidots

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
    // ubidots.setDebug(true);                    // uncomment this to make debug messages available
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

    // Recall from storage
    if(EEPROM.read(NUMB_OF_SLAVES_ADDRESS) != 255)          
      numbOfSlaves = EEPROM.read(NUMB_OF_SLAVES_ADDRESS);   // Recall how many slaveUnits is addressed
    for(int slaveUnit=0; slaveUnit < numbOfSlaves; slaveUnit++)
    {
      macSlave[slaveUnit] = EEPROM.readString(MAC_ADDRESS_SLAVE_START + SIZE_OF_MAC*slaveUnit);   // Recall all stored mac addresses
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

  // Fool program to think it has been woken and should publish data
  timerWakeup = true;
  bool okToSleep = true;                          // Will be set to false during loop by program parts that need it
  
  if (S2.read()) {buttonTS = millis(); buttonPress = true;} 

  if (buttonPress) {
    display.enable();
    display.selectLockScreen();
    display.refresh();
  }

  // millis() rollover check
  if(millis() < millis_prev)  millisRollover = true;
  else                        millisRollover = false;

  // hour() change check. Do not set false if not, because it will be an indicator that an hour change has been ,ade during the process and that data shoiuld be transmitted
  if(hour() != hour_prev) hourChange = true;

  // Update data from unit
  dataIsReady = dataFetchLoop((hour() == gpsCheckHour1)||(hour() == gpsCheckHour2));    // Get gps data only when the hour says 12. Returns true when requested data is updated

  // Update display
  displayLoop();
  
  // If year is not updated, do not continue until GPS has updated time and date (To be able so sync units)
  if ((year()<= 2020) && (gpsReconAttempts > 0)){
    
    int scan = 0;
    int totScan = 30 * 10;                        // Run loop for 30 seconds to see if GPS data is recieved
    delay(1000); 
     
    while ((year()<= 2020) && gps.isEnabled()){
      
      if (S2.read()) {buttonTS = millis(); buttonPress = true;}          // Read state of pushbutton
      if (S2.isPressed()) break;                  // Break if button has been pressed

      commLoop();                                 // Maintain communication with slaves

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

      if (totScan <= 2){
        okToSleep = false;                        // Cannot go to sleep since time not ok
        display.clearStrings();
        gpsReconAttempts--;                       // Decrement amount of remaining attempts
        break;
      }
      totScan--;
      delay(0);
    }
  }

  

  /*******************************************************
  * Communicate data between units and publish to Ubidots
  ********************************************************/
  
  ////////////////////
  //     MASTER     //
  ////////////////////

  if (master){

    // Listen for incoming data and scan local area for new slaves
    commLoop();
     
    // Is triggered either by periodical wakeuptimer or by an hour change if wakeup was triggered by button
    if(timerWakeup || hourChange){
      
      #ifdef DEBUG 
        Serial.println("Master attempting to get data from stored Slaves"); 
      #endif
      
      // Start gathering data and publish to ubidots

      // UBIDOTS - Check if connected and reconnect if not
      ubidots.loop();
      if (!ubidots.connected()) ubidots.reconnect();

      // Publish Master data if not allready done. Make 5 attempts with 1 second spacing
      if (!masterDataPublished && dataIsReady){
        for (int i = 0; i<5; i++){
          if (masterDataPublished) break;
          masterDataPublished = ubiPubData(ubidotsId, sensorData);
          delay(1000);
        }
      } 

      // Fetch data from slaves 
      uint16_t timeout = 120 * uS_TO_S_FACTOR; //Leave 2 minutes to slaves to fetch data

      // Don't fetch data right after boot,
      // and check if any slaves are set up
      if((millis() - setupTime) > timeout && numbOfSlaves && retryFetchSlaveData){ 
      
        /* // Fetch data from slave units
          -- Request data from the given slave number
          -- Wait for data from the given slave
          -- If no data recieved, stash unit number in an array, 
          -- to request again at the end of the original list
          -- Push data to the cloud
        */
      
        bool resivedData[numbOfSlaves] = {1};
        for (int slaveUnit = 0; slaveUnit < numbOfSlaves; slaveUnit++){

          float slaveData[7];   // Temp[0] Hum[1] Press[2] Lat[3] Lng[4] Alt[5] BatteryPercent[6]
          
          String slaveID = macSlave[slaveUnit];
          if(slaveID == "FF:FF:FF:FF:FF:FF") continue;    // Skip unit if it is deleted

          
        
          // If error, store id for a second attempt and move on to next slaveUnit
          if(!getSlaveData(slaveUnit, slaveData)) { resivedData[slaveUnit] = false; continue; }  

          unsigned long TS = millis();
          while(!ubiPubData(slaveID, slaveData))          // Wait on data to be pushed to ubidots
          {
            if((millis()-TS) > ubiPubFreq*2)              // If timeoute exit loop, with error
            {
              Serial.println("ERROR: can not publish to Ubidots");
              break;
            } 
          }
        }


        // Do a secound attempt to get data from slave units
        uint8_t slaveUnit = 0;

        for(bool state : resivedData)   // Loop through all elements in resive data to see if data was fetched
        {
          slaveUnit++;                  // Increment unit to use as index
          if(state) continue;           // Move on to next devise

          float slaveData[7];           // Temp[0] Hum[1] Press[2] Lat[3] Lng[4] Alt[5] BatteryPercent[6]

          String slaveID = macSlave[slaveUnit];

          if(!getSlaveData(slaveUnit, slaveData)) continue;   // Move on if an error occured while to fetching data

          unsigned long TS = millis();
          while(!ubiPubData(slaveID, slaveData))              // Wait on data to be pushed to ubidots
          {
            if((millis()-TS) > ubiPubFreq*2)                  // If timeoute exit loop, with error
            { 
              Serial.println("ERROR: can not publish to Ubidots"); 
              break; 
            } 
          }
          resivedData[slaveUnit] = true;    // Force read state to read data (ok)
        }
        
        fetchSlaveDataAttempts --;           
        if (fetchSlaveDataAttempts < 1){    // Check it max numder of retryes is reached
          retryFetchSlaveData = false;      // Stop retrying
        }
      }
    }
  }
  ////////////////////
  //      SLAVE     //
  ////////////////////

  if (!master){
    
    // Attempt to get ID from master if none is given
    if (CComErr && getIDAttempts > 0) {
      Serial.println("Com err");
      display.selectMessageScreen();
      display.setString(0, "CCom");
      display.setString(2, "Obtaining ID");
      display.setString(3, "from \"Master\"");
      display.refresh();
      Serial.println("Attempting to obtain ID from Master");
      
      CComErr = !CCom.getId();
      
      if (CComErr)  {display.setString(5, "Failed to"); display.setString(6, "obtain ID"); okToSleep = false;}
      else          {display.setString(5, "ID obtained"); display.setString(6, "successfully");}
      display.refresh();
      delay(1000);

      // Go back to lock screen
      display.clearStrings();
      display.selectLockScreen();
      
      display.refresh();
      delay (500);
      getIDAttempts--;
    }

    commLoop();                                     // Run com loop when button has booted unit, so it can associate with master/slave
  }


  /////////////////////
  //  END PROCEDURE  //
  /////////////////////

  // Decide if unit shoud be put into deep sleep or only shut off display
  
  // Sleep enable 
  if (okToSleep){
    
    // Only valid if the buttonpush triggered the wakeup
    if (!hourChange && !timerWakeup && (millis() - buttonTS > awakeTime)){
      Serial.println("Sending unit to deep sleep based on buttonpush wakeup");
      display.disable();                          // Turn display off to save power
      goToDeepSleep();                            // Enter deep sleep. Wakeup calculated to be at next full hour
      }    // Go to deep sleep if no other activity
    
    // Disable display upon lack of buttonpress
    else if (display.isEnabled() && (millis() - buttonTS > awakeTime)){
      display.disable(); buttonPress = false;     // Only turn off display if timer runs out but other activity
      Serial.println("Disabling display");        // Give feedback via serial that unit is disabling display on purpose
    }
    
    // Enable display upon buttonpress
    else if (!display.isEnabled() && (millis() - buttonTS < awakeTime)){
      display.enable();                           // Turn display back on because of activity
      Serial.println("Enabling display");         // Give feedback via serial that unit is enabling display on purpose
    }
  
    // Go to deep sleep if unit has attempted to get data from other units for x amount of seconds
    if ((hourChange || timerWakeup) && ((millis() - setupTime > timeDataFetchSlaves) || millisRollover)){
      Serial.println("Sending unit to deep sleep");
      display.disable();                          // Turn display off to save power
      goToDeepSleep();                            // Enter deep sleep. Wakeup calculated to be at next full hour
    }
  }

  if (S2.read()) {buttonTS = millis(); buttonPress = true;}                            
  // Factory reset and reboot
  if(S2.pressedFor(2000)){
    display.selectMessageScreen();
    display.setString(0, "Reset unit");
    display.setString(2, "To reset unit,");
    display.setString(3, "keep pressing");
    display.setString(4, "right button");
    display.refresh();
    
    uint8_t scanTot = 8*6;
    uint8_t scanShort = 0;
    while (S2.isPressed()){
      
      if (S2.read()) {buttonTS = millis(); buttonPress = true;}         // Read S2 and update timestamp for when button was last active

      // Display oading symbol
      if      (scanShort == 0) display.setString(6, "|");
      else if (scanShort == 1) display.setString(6, "/");
      else if (scanShort == 2) display.setString(6, "-");
      else if (scanShort == 3) display.setString(6, "\\");

      if (scanShort < 3) scanShort++;
      else scanShort = 0;

      // Display countdown
      if      (scanTot > 5*8) display.setString(8, "6");
      else if (scanTot > 4*8) display.setString(8, "5");
      else if (scanTot > 3*8) display.setString(8, "4");
      else if (scanTot > 2*8) display.setString(8, "3");
      else if (scanTot > 1*8) display.setString(8, "2");
      else if (scanTot > 0)   display.setString(8, "1");

      display.refresh();

      // Show message on display, verifying that unit will be reset to factory settings
      if (scanTot <= 0){
        display.setString(1, "");
        display.setString(2, "");
        display.setString(3, "Resetting unit");
        display.setString(4, "to factory");
        display.setString(5, "settings");
        display.setString(6, "");
        display.setString(8, "");
        display.refresh();

        delay(10);

        // Wipe allocated EEPROM
        for (int i = 0; i < EEPROM_SIZE; i++) {
          EEPROM.write(i, 255);
        }
        if(EEPROM_COMMIT) EEPROM.commit();
        delay(500); 
        ESP.restart();                           // Reboot unit after EEPROM is wiped
      }
      scanTot --;
      delay(125);
    }
  }
  display.clearStrings();                         // Clear strings set in display object to avoid bugs

  // Disable GPS if no longer needed
  if (gps.isEnabled() && dataIsReady && (year() > 2020)){
    gps.disable();
  }

  Serial.println("Loop active");

  // End procedure. Check for hour change and millis rollover
  millis_prev = millis();
  hour_prev = hour();
  delay(0);                                       // Scan frequency

  
}


