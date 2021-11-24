
#pragma once

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>

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
        float lng;                          // Longitudal coordinates
        float alt;                          // Altitude
        unsigned long locUpdateTS;          // Timestamp for last location update

        // Date
        long yyyy;                          // Year
        int mm;                             // Month
        int dd;                             // Day of month
        unsigned long dateUpdateTS;         // Timestamp for last date update

        // Time
        int hour;                           // Hour
        int min;                            // Minutes
        int sec;                            // Second
        int ms;                             // Millisecond
        unsigned long timeAge;              // 
        unsigned long timeUpdateTS;         // Timestamp for last time update
    public:

        // Constructor 
        GPS(int baudGPS, long baudSerial, int pinPS, int pinTx, int pinRx); // Baud, PinPS, PinTX, PinRx

        // Unit control
        
        // Power up and start communication with GPS module
        void enable();   

        // Update stored value for GPS data (loc, alt, date, time)                   
        void refresh();  

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



        // Synchronizing internal ESP32 clock with satellite
        void syncDateTime();                

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

class PlantSurveilance{

    private:

    public:

};




