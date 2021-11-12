
#pragma once


class GPS{
    
    private:
    
    // GPS
    int GPSPsPin;
    int GPSRxPin;
    int GPSTxPin;
    int GPSBaud;
    
    public:

    // Constructor 
    GPS(int baud, int pinPS, int pinTx, int pinRx); //
    


    void setGPSPsPin(int);              // Set pin for power supply to GPS sensor
    void setGPSRxPin(int);              // Set pin for serial reception
    void setGPSTxPin(int);              // Set pin for serial transmission

    void configure();
    void enable();                      // Start serial communication with 
    int getLatitude();                  
    int getLongitude();

};



