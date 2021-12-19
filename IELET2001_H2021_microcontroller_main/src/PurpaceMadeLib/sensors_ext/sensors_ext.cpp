
#include "sensors_ext.h"                        // Header file for this .cpp

#include <TinyGPS++.h>                          // GPS lib
#include <SoftwareSerial.h>                     // Software based serial com
#include <TimeLib.h>                            // Time lib
#include <Adafruit_BME280.h>                    // Temp, hum, press sensor lib


//////////////////////////////////////////////////////
//                                                  //
//                        GPS                       //
//                                                  //
//////////////////////////////////////////////////////

// Constructor
GPS::GPS(int baudGPS, long baudSerial, int pinPS, int pinTx, int pinRx):
    ssGPS(pinRx, pinTx),                        // Instanciate SoftwareSerial object
    gps_unit()                                  // Instanciate gps object
    {       
    
    // Read pins and baud from constructor to object
    GPSPsPin =  pinPS;
    GPSTxPin =  pinTx;
    GPSRxPin =  pinRx;
    GPSBaud =   baudGPS;

    // Refresh function parameters
    enableStartTS =     0;
    enableTime =        10000;                  // Give GPS up to 10 seconds to connect 
    refreshStartTS =    0;                      // Initially 0
    refreshTime =       150;                    // Time spent reading data from GPS

    // Pre-define coordinates and alt to zero-values
    lat =       0;
    lng =       0;
    alt =       0;

    // Pre-define time and date to zero-values
    yyyy =      1970;
    mm =        1;
    dd =        1;

    hour =      0;
    min =       0;
    sec =       0;
    ms =        0;

    enabled = false;

    // Define the pin powering the GPS module as an output
    
    pinMode(GPSPsPin, OUTPUT);

    Serial.begin(baudSerial);

};

// Enable and disable gps module

void GPS::enable(){

    enabled = true;

    enableStartTS = millis();                  // Set timestamp for function call
    
    #ifdef DEBUG
        Serial.println("Enabling GPS module");
    #endif
    
    digitalWrite(GPSPsPin, HIGH);               // Powering up GPS module
    
    #ifdef DEBUG
        Serial.println("GPS module powered up");
    #endif
    
    ssGPS.begin(GPSBaud);                       // Establish software serial for GPS with given baud rate

    // Wait for 10 seconds to see if serial communication for GPS becomes available
    while ((millis() - enableStartTS < enableTime) && !ssGPS.available()){
        // Wait to see if serial com is set up successfully
    }
    
    // Feedback saying if serial communication is set up sucessfully or not
    if (ssGPS.available()){
        enabled = true;
        #ifdef DEBUG
            Serial.println("Serial communication set up successfully");
        #endif
    }
    else{
        #ifdef DEBUG
            Serial.println("Serial communication not set up sucessfully");
        #endif
    }
    

};

bool GPS::isEnabled(){
    return enabled;
}

void GPS::refresh(bool syncDateTime = HIGH){

    // Set all temporary values to 0
    _yyyy   = 0;
    _mm     = 0;
    _dd     = 0;

    _hour   = 0;
    _min    = 0;
    _sec    = 0;

    _lat    = 0;
    _lng    = 0;
    _alt    = 0;

    _gpsDataAvailable = LOW;

    bool _dateOk    = false;                    // Date read OK
    bool _timeOK    = false;                    // Time read OK
    bool _posOk     = false;                    // Position OK
    bool _altOk     = false;                    // Altitude read


    refreshStartTS = millis();                  // Set timestamp for function call
    
    /*--------------------------------------------------------------------
                                    GPS
    --------------------------------------------------------------------*/


    // Spend [refreshTime] milliseconds on reading and encoding data from GPS module
    do{
        while (ssGPS.available()){
            if (ssGPS.peek() != 0){
                _gpsDataAvailable = HIGH;
            }
            gps_unit.encode(ssGPS.read());
        }
    } while (millis() - refreshStartTS < refreshTime);


    // Update - Location
    if ((_gpsDataAvailable) && (gps_unit.location.isValid())){           
        _lat = gps_unit.location.lat();          // Fetch latitude from satellite
        _lng = gps_unit.location.lng();          // Fetch longitude from satellite
        _alt = gps_unit.altitude.meters();       // Fetch altitude from satellite
    }

    /* 
    Since the chance of the unit being placed at Null Island equals 0, this is a good
    indicator that the GPS data is not read sucessfully and should not be stored as a
    valid value*/
    if ((_lat != 0.0) && (_lng != 0.0)){ 
        // GPS loc is valid
       
        // Push temp values to stored values
        lat = _lat;                             
        lng = _lng;
        alt = _alt; 
        #ifdef DEBUG 
            Serial.println("Update successful: Location");
        #endif
        _posOk = true;
    }
        
    else {
        #ifdef DEBUG
            Serial.println("Update failed:     Location");
        #endif
    }


    /*--------------------------------------------------------------------
                                TIME AND DATE
    --------------------------------------------------------------------*/

    // Update - Date
    if ((_gpsDataAvailable)&&(gps_unit.date.isValid())){

        // Save fetched values in temporary variables
        _yyyy =  gps_unit.date.year();           // Fetch year from satellite
        _mm =    gps_unit.date.month();          // Fetch month from satellite
        _dd =    gps_unit.date.day();            // Fetch day of month from satellite
        _dateOk = true;    
    }

    
    // Update - Time
    if ((_gpsDataAvailable)&&(gps_unit.time.isValid())){
        
        // Save fetched values in temporary variables
        _hour   =   gps_unit.time.hour();       // Fetch hour from satellite
        _min    =   gps_unit.time.minute();     // Fetch minute from satellite
        _sec    =   gps_unit.time.second();     // Fetch second from satellite 

        _timeOK = true;     
    }


    // Update all values if a real date is read
    if ((_dd != 0)&&(_mm != 0)) {               // Since date cannot be zero, this is an indicator 
        
        // Push temp values to stored values
        yyyy = _yyyy;
        mm = _mm;
        dd = _dd;
        
        hour = _hour;
        min = _min;
        sec = _sec;

        #ifdef DEBUG
            Serial.println("Update successful: Time and date");
        #endif

        if(syncDateTime){
            
            setTime(hour, min, sec, dd, mm, yyyy);
            adjustTime(timezoneOffset * SECS_PER_HOUR);
            #ifdef DEBUG
                Serial.println("Time and date synchronized");
                Serial.print("Current time: ");
                Serial.print(hour);
                Serial.print(":");
                Serial.print(minute());
                Serial.print(":");
                Serial.print(second());
                Serial.print(" ");
                Serial.print(day());
                Serial.print(" ");
                Serial.print(month());
                Serial.print(" ");
                Serial.print(year()); 
                Serial.println(); 
            #endif

        }
    }

    else{
        #ifdef DEBUG
            Serial.println("Update failed:     Time and date");
        #endif
    }


    // Give overview of chars processed
    
    #ifdef DEBUG
        Serial.print("Chars processed:   ");
        Serial.println(gps_unit.charsProcessed());
    #endif

}

void GPS::disable(){

    enabled = false;
    
    #ifdef DEBUG
        Serial.println("Disabling GPS module");
    #endif
    
    ssGPS.end();                                // Shut down serial com to GPS module
    
    #ifdef DEBUG
        Serial.println("GPS serial communication ended");
    #endif
    
    digitalWrite(GPSPsPin, LOW);                // Power down GPS sensor
    
    #ifdef DEBUG    
        Serial.println("GPS module powered down");
    #endif
};

// Get functions - POSITION

float GPS::getLatitude(){
    return lat;
};

float GPS::getLongitude(){
    return lng;
};

float GPS::getAltitude(){
    return alt;
}

// Get functions - DATE

long GPS::getYear(){
    return yyyy;
}

int GPS::getMonth(){
    return mm;
}

int GPS::getDay(){
    return dd;
}

// Get functions - TIME

int GPS::getHour(){
    return hour;
}

int GPS::getMinute(){
    return min;
}

int GPS::getSecond(){
    return sec;
}

unsigned long GPS::getTimeAge(){
    return timeAge;
}




//////////////////////////////////////////////////////
//                                                  //
//                  WEATHER STATION                 //
//                                                  //
//////////////////////////////////////////////////////

WeatherStation::WeatherStation(int i2cAdr):
    bme280()                                    // Member temp, hum, press sensor
    {
    _i2cAdr = i2cAdr;                           // I2C adress for BME280
    _nRead = 100;                               // Number of readings per measurement (to get avg)

}

void WeatherStation::enable(){
    
    #ifdef DEBUG
        Serial.println("Enabling Weather station"); 
    #endif 

    bme280.begin(_i2cAdr);                      // Establish I2C communication with BME280


}

void WeatherStation::refresh(){

}


float WeatherStation::getTempC(){
    
    // Set reading sum to 0
    float _tempAcc = 0;
    
    // Do [_nRead] measurements
    for (int i = 0; i < _nRead; i++){
        _tempAcc += bme280.readTemperature();
    };

    // Devide sum on amount of measurements to get avg
    float _tempAvg = (_tempAcc / _nRead);

    // Return average temperature
    return _tempAvg;

};

float WeatherStation::getHum(){
    
    // Set reading sum to 0
    float _humAcc = 0;
    
    // Do [_nRead] measurements
    for (int i = 0; i < _nRead; i++){
        _humAcc += bme280.readHumidity();
    };

    // Devide sum on amount of measurements to get avg
    float _humAvg = (_humAcc / _nRead);

    // Return average temperature
    return _humAvg;

};

float WeatherStation::getPressHPa(){
    
    // Set reading sum to 0
    float _pressAcc = 0;
    
    // Do [_nRead] measurements
    for (int i = 0; i < _nRead; i++){
        _pressAcc += bme280.readPressure();
    };

    // Devide sum on amount of measurements to get avg
    float _pressAvgPa = (_pressAcc / _nRead);   // Avg pressure [Pa]
    float _pressAvgHPa = _pressAvgPa / 100.0;   // Convert from [Pa] to [hPa]

    // Return average temperature
    return _pressAvgHPa;

};




//////////////////////////////////////////////////////
//                                                  //
//                     UNIT DATA                    //
//                                                  //
//////////////////////////////////////////////////////

UnitData::UnitData(int pinBattery){ // Constructor
    
    // Battery
    batteryReadPin = pinBattery;
    
}

void UnitData::enable(){

    // Set pinmode for battery surveilance pin
    pinMode(batteryReadPin, INPUT);

}

void UnitData::refresh(){

    // Battery
    batteryPercent = map(analogRead(batteryReadPin), 1750, 2550, 0, 100);
    batteryVoltage = map(analogRead(batteryReadPin), 1750, 2550, 0, 3.3);

}

int UnitData::getBatteryPercent(){
    return(batteryPercent);
}

float UnitData::getBatteryVoltage(){
    return(batteryVoltage);
}