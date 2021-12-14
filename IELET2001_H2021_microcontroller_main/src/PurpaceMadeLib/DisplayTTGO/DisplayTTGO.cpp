
#include "DisplayTTGO.h"


// Class containing everything going on on the built-in display on the microcontroller



DisplayTTGO::DisplayTTGO():
tft(135, 240)
{
    tft.init();
    
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(0);
    tft.setTextSize(1);
    tft.setTextColor(TFT_CYAN);
    
};


void DisplayTTGO::showLockScreen(){
    
    /*
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(67, 67);
    
    tft.setTextColor(TFT_CYAN);

    tft.print("Test");
    */
    tft.fillScreen(TFT_BLACK);
    drawBatteryState(95);



};

void DisplayTTGO::drawBatteryState(int percent){
    
    int batteryPercent = percent;

    // Draw battery shape in top right corner of screen
    tft.drawRoundRect(105,4,24,10,1,TFT_CYAN);
    tft.fillRoundRect(103,6,2,6,0,TFT_CYAN);


    // Show "cells" in battery based on power level
    if (batteryPercent > 67){
        tft.fillRect(107,6,6,6,TFT_CYAN);
    };
    if (batteryPercent > 33){
        tft.fillRect(114,6,6,6,TFT_CYAN);
    };
    if (batteryPercent > 10){
        tft.fillRect(121,6,6,6,TFT_CYAN);
    }
    else if (batteryPercent <= 10){ 
        tft.fillRect(121,6,6,6,TFT_RED);
    };


    char percentString[20] = "";
    sprintf(percentString, "%i", batteryPercent);



    int x = 100;
    int y = 1;
    int font = 2;
    tft.drawRightString(percentString, x, y, font);



   
};
