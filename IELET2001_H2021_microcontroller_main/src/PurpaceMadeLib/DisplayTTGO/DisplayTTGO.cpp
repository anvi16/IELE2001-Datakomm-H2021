
#include "DisplayTTGO.h"


// Class containing everything going on on the built-in display on the microcontroller



DisplayTTGO::DisplayTTGO():
tft(135, 240)
{
    tft.init();
    tft.setRotation(4);
    


};



void DisplayTTGO::showLockScreen(){
    
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(67, 67);
    
    tft.setTextColor(TFT_CYAN);

    tft.print("Test");
    drawBatteryState(4);



};

void DisplayTTGO::drawBatteryState(int percent){
    /*
    tft.setTextSize(1);
    tft.setCursor(80, 6);
    tft.print(percent);
    tft.print("%");
    */

    tft.drawRoundRect(110,4,24,10,1,TFT_CYAN);
    tft.fillRoundRect(108,6,2,6,0,TFT_CYAN);

    if (percent > 67){
        tft.fillRoundRect(112,6,6,6,0,TFT_CYAN);
    }

    if (percent > 33){
        tft.fillRoundRect(119,6,6,6,0,TFT_CYAN);
    }
    
    if (percent > 10){
        tft.fillRoundRect(126,6,6,6,0,TFT_CYAN);
    }
    
    else if (percent < 11){
        
        tft.fillRoundRect(126,6,6,6,0,TFT_RED);
    }
};
