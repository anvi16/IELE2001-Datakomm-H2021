
#pragma once

#include <TFT_eSPI.h>                           // TFT lib for control of OLED screen
#include <SPI.h>                                // SPI lib. TFT lib depends on this
#include <TimeLib.h>                            // Time lib

class DisplayTTGO{
    
    private:

        int displayNumber;
            /* 
            0 - Disable
            1 - Lock screen
            2 - Message Screen
        */

        bool master;
        int batteryPercent;
        float batteryVoltage;

        // Data to be displayed on lock screen
        float lockScreenData[6];                // Temp, hum, press, lat, lng, alt
        float lockScreenData_prev[6];           // Temp, hum, press, lat, lng, alt
        
        // Vars for comparing updates between scans
        int min_prev;                           // Time update
        int batteryPercent_prev;                // Battery update
        int batteryVoltage_prev;                // Battery update
        int displayNumber_prev;                 // Screen update

        // Strings to be displayed
        String str[10] = {};                    // 0 = Header, rest = line
        String str_prev[10] = {};               // 0 = Header, rest = line
        
        // Hardware pin
        int _pinBL;

        TFT_eSPI tft;
        void drawBatteryState(int percent);
        void drawTime();
        void drawMessageScreenStrings();
        void drawMaster();                      // Draw master symbol

    public: 
        // Constructor
        DisplayTTGO(int pinBL);                 // Constructor
        

        // Different displays
        void selectBlackScreen();               // 0
        void selectLockScreen();                // 1
        void selectMessageScreen();             // 2


        // Refresh function to update all elements on screen. Call each scan
        void refresh();

        // Update battery stats
        void setBatteryState(int percent, float volt);
        
        // Update data to be displayed on lock screen
        void setLockScreenData(float t, float h, float p, float lat, float lng, float alt);

        // Define a string to be displayed in a certain pos when refresh() is called and MessageScreen is active
        void setString(int nLine, String text); // Define string to be called in function
        
        // Clear all strings saved in object
        void clearStrings();                    // Clear all saved strings

        // Inform that unit is configured as master
        void setMaster();
        

        

        










};