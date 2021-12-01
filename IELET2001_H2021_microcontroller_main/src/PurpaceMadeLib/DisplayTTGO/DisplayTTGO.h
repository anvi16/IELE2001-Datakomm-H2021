
#pragma once

#include <TFT_eSPI.h>                           // TFT lib for control of OLED screen
#include <SPI.h>                                // SPI lib. TFT lib depends on this


class DisplayTTGO{
    
    private:

        TFT_eSPI tft;
    
        void drawBatteryState(int);


    public:

        DisplayTTGO();          // Constructor
        
        void showLockScreen();
        
        void refresh();

        

        










};