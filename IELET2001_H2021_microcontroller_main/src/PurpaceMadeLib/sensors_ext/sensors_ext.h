
#pragma once


#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <TimeLib.h>

#ifndef DEBUG
#define DEBUG
#endif


class GPS{
    
    private:
    
        // Declare member classes
        SoftwareSerial ssGPS;               // Software serial
        TinyGPSPlus gps_unit;               // The TinyGPS++ object

        // GPS
        int GPSPsPin;                       // GPS module power supply pin
        int GPSRxPin;                       // GPS serial communication recieve pin
        int GPSTxPin;                       // GPS serial communication transmit pin
        int GPSBaud;                        // GPS serial communication baud rate

        unsigned long enableStartTS;        // Timestamp showing when enable() func was called
        unsigned long enableTime;           // Time given to enable() function to sucessfully establish com
        unsigned long refreshStartTS;       // Timestamp showing when refresh() func was called
        unsigned long refreshTime;          // Time refresh function should run (to read from GPS) 
        

        // Coordinates
        float lat;                          // Latitudal coordinates
        float _lat;                         // Latitudal coordinates - Temp val
        float lng;                          // Longitudal coordinates
        float _lng;                         // Longitudal coordinates - Temp val
        float alt;                          // Altitude
        float _alt;                         // Altitude - Temp val

        bool _gpsDataAvailable;

        unsigned long locUpdateTS;          // Timestamp for last location update

        // Date
        long yyyy;                          // Year
        long _yyyy;                         // Year - Temp val
        int mm;                             // Month
        int _mm;                            // Month - Temp val
        int dd;                             // Day of month
        int _dd;                            // Day of month - Temp val
        unsigned long dateUpdateTS;         // Timestamp for last date update

        // Time
        int hour;                           // Hour
        int _hour;                          // Hour - Temp val
        int min;                            // Minutes
        int _min;                           // Minutes - Temp val
        int sec;                            // Second
        int _sec;                           // Second - Temp val
        int ms;                             // Millisecond
        int _ms;                            // Millisecond - Temp val
        unsigned long timeAge;              // 
        unsigned long timeUpdateTS;         // Timestamp for last time update
  
        const int timezoneOffset = 0;       // Central European Time


    public:

        // Constructor 
        GPS(int baudGPS, long baudSerial, int pinPS, int pinTx, int pinRx); // Baud, PinPS, PinTX, PinRx

        // Unit control
        
        // Power up and start communication with GPS module
        void enable();   

        // Update stored value for GPS data (loc, alt, date, time)
        // When syncTimeAndDate is set HIGH, the internal time and
        // date of the microcontroller will be synced to a satellite 
        // when signal is read successfully                
        void refresh(bool syncDateTime);  

        // Power down GPS module to save power                   
        void disable();                     
        



        //-//-// Coordinates //-//-//

        // Reads lat coordinates with 6 decimal points accuracy
        float getLatitude();

        // Reads long coordinates with 6 decimal points accuracy                             
        float getLongitude();   

        // Reads altutude of sensor            
        float getAltitude();                




        //-//-// Date //-//-//    

        // Returns current year
        long getYear();       

        // Returns current month              
        int getMonth();   

        // Returns current day of month
        int getDay();                       
        
        
        
        
        //-//-// Time //-//-//

        // Returns current hour
        int getHour();  

        // Returns current minute                    
        int getMinute();    

        // Returns current second                
        int getSecond();

                            
        unsigned long getTimeAge();
              

};

class WeatherStation{

    private:
        
        Adafruit_BME280 bme280;

        // I2C communication
        int _i2cAdr;                        // I2C Adress dor BME280

        // Reading parameters
        int _nRead;                         // Number of readings per measurement (to get avg)
                  


    public:
        // Constructor - Takes (i2cSDA pin, i2cSCL pin)
        WeatherStation(int);


        
        // Start communication with BME280 sensor
        void enable();

        // Update readings from weather station
        void refresh();



        // Returns an average temperature [C] based on [nRead] readings
        float getTempC();         

        // Returns an average humidity [%] based on [nRead] readings
        float getHum();      

        // Returns an average pressure [hPa] based on [nRead] readings
        float getPressHPa(); 

                         



};

class UnitData{
    private:

        // Batterystate
        int batteryReadPin;
        int batteryVoltage;
        int batteryPercent;

    public:
        UnitData(int pinBattery); // Constructor
        void enable();
        void refresh();
        int getBatteryPercent();

};

struct WeatherData{

    int id;

    float temp;
    float hum;
    float press;
    float lat;
    float lng;
    float alt;

};

class PlantSurveilance{

    private:

    public:

};




