
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
            2 - Master / slave menu
            */

        // Vars for comparing updates between scans
        int min_prev;                           // Time update
        int batteryPercent_prev;                // Battery update
        int displayNumber_prev;                 // Screen update
        
        // Hardware pin
        int _pinBL;

        TFT_eSPI tft;
        void drawBatteryState(int percent);
        void drawTime();


    public: 
        // Constructor
        DisplayTTGO(int pinBL);                 // Constructor
        

        // Different displays

        void selectBlackScreen();               // 0
        void selectLockScreen();                // 1
        void selectMasterSlaveMenu();           // 2

        // Refresh function to update all elements on screen. Call each scan
        void refresh(int batteryPercent);

        

        










};