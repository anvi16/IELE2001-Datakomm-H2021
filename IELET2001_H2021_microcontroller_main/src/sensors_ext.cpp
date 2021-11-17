
#include "sensors_ext.h"                        // Header file for this .cpp

#include <TinyGPS++.h>                          // GPS lib
#include <SoftwareSerial.h>                     // Software based serial com
#include <time.h>                               // Time lib
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
    refreshTime =       2000;                   // 2 seconds

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


    // Define the pin powering the GPS module as an output
    pinMode(GPSPsPin, OUTPUT);

    Serial.begin(9600);

};

// Enable and disable gps module

void GPS::enable(){

    refreshStartTS = millis();                  // Set timestamp for function call
    
    Serial.println("Enabling GPS module");
    digitalWrite(GPSPsPin, HIGH);               // Powering up GPS module
    Serial.println("GPS module powered up");
    ssGPS.begin(GPSBaud);                       // Establish software serial for GPS with given baud rate


    // Wait for 10 seconds to see if serial communication for GPS becomes available
    while ((millis() - enableStartTS < enableTime) && !ssGPS.available()){
        // Wait to see if serial com is set up successfully
    }
    
    // Feedback saying if serial communication is set up sucessfully or not
    if (ssGPS.available()){
        Serial.println("Serial communication set up successfully");
    }
    else{
        Serial.println("Serial communication not set up sucessfully");
    }
    

};

void GPS::refresh(){
    
    refreshStartTS = millis();                  // Set timestamp for function call
    
    // Spend [refreshTime] milliseconds on reading and encoding data from GPS module
    do
    {
        while (ssGPS.available()){
            gps_unit.encode(ssGPS.read());
        }
    } while (millis() - refreshStartTS < refreshTime);

    // Update - Location
    if (gps_unit.location.isValid()){           
        lat = gps_unit.location.lat();          // Fetch latitude from satellite
        lng = gps_unit.location.lng();          // Fetch longitude from satellite
        alt = gps_unit.altitude.meters();       // Fetch altitude from satellite
        Serial.println("Update successful: Location");
    }
    else {
        Serial.println("Update failed:     Location");
    }
    
    // Update - Date
    if (gps_unit.date.isValid()){
        yyyy =  gps_unit.date.year();           // Fetch year from satellite
        mm =    gps_unit.date.month();          // Fetch month from satellite
        dd =    gps_unit.date.day();            // Fetch day of month from satellite
        Serial.println("Update successful: Date");
    }
    else {
        Serial.println("Update failed:     Date");
    }

    // Update - Time
    if (gps_unit.time.isValid()){
        hour =  gps_unit.time.hour();           // Fetch hour from satellite
        min =   gps_unit.time.minute();         // Fetch minute from satellite
        sec =   gps_unit.time.second();         // Fetch second from satellite
        timeAge = gps_unit.time.age();
        Serial.println("Update successful: Time");
    }
    else {
        Serial.println("Update failed:     Time");
    }

    // Give overview of chars processed
    Serial.print("Chars processed:   ");
    Serial.println(gps_unit.charsProcessed());

}

void GPS::disable(){
    
    Serial.println("Disabling GPS module");
    ssGPS.end();                                // Shut down serial com to GPS module
    Serial.println("GPS serial communication ended");
    digitalWrite(GPSPsPin, LOW);                // Power down GPS sensor
    Serial.println("GPS module powered down");
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

void GPS::syncDateTime(){
    
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
    
    Serial.println("Enabling Weather station");                           
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