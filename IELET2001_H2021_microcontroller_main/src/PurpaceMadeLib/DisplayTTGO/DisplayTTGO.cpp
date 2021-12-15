#include "DisplayTTGO.h"


// Class containing everything going on on the built-in display on the microcontroller


// Constructor
DisplayTTGO::DisplayTTGO(int pinBL):
tft(135, 240)
{   
    _pinBL = pinBL; // Backlight pin

    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(0);
    tft.setTextSize(1);
    tft.setTextColor(TFT_CYAN);

    batteryPercent_prev = 0;    // Init
    min_prev = 59;              // Init
    
};

// Different displays
void DisplayTTGO::selectBlackScreen(){              // 0
    displayNumber = 0;
}
void DisplayTTGO::selectLockScreen(){               // 1
    displayNumber = 1;
}
void DisplayTTGO::selectMasterSlaveMenu(){          // 2
    displayNumber = 2;
}




void DisplayTTGO::drawBatteryState(int percent){
    
    percent = 100;

    tft.fillRect(76,4,59,10,TFT_BLACK);

    tft.drawRoundRect(110,4,24,10,1,TFT_WHITE);
    tft.fillRect(108,6,2,6,TFT_WHITE);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(101, 6);
    tft.drawNumber(percent, 76, 1, 2);
    tft.print("%");

    if (percent > 67){
        tft.fillRect(112,6,6,6,TFT_WHITE);
    }

    if (percent > 33){
        tft.fillRect(119,6,6,6,TFT_WHITE);
    }
    
    if (percent > 10){
        tft.fillRect(126,6,6,6,TFT_WHITE);
    }
    
    else if (percent < 11){
        
        tft.fillRect(126,6,6,6,TFT_RED);
    }
};

void DisplayTTGO::drawTime(){
    int _hour = hour();
    int _min = minute();
    int xpos = 5;
    int ypos = 1;

    tft.fillRect(1, 1, 42, 15, TFT_BLACK);

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    if (_hour < 10) xpos += tft.drawChar('0', xpos, ypos, 2);   // Add hours leading zero for 24 hr clock
    xpos += tft.drawNumber(_hour, xpos, ypos, 2);               // Draw hours
    xpos += tft.drawChar(':', xpos, ypos, 2);
    if (_min < 10) xpos += tft.drawChar('0', xpos, ypos, 2);    // Add minutes leading zero
    xpos += tft.drawNumber(_min, xpos, ypos, 2);    

}

void DisplayTTGO::refresh(int batteryPercent){

    // Turn display backlight ON if BlackScreen has previously been displayed
    if ((displayNumber_prev == 0) && (displayNumber != 0)){
        digitalWrite(_pinBL, HIGH);
    }

    // Check if minute has been updated since last scan
    if (minute() != min_prev){
        drawTime();
        tft.drawLine(2, 17, 133, 17, TFT_WHITE);
    }

    // Refresh battery status
    if (batteryPercent != batteryPercent_prev){
        drawBatteryState(batteryPercent);
    }


    



    /****************************************
    * Define different screen specs
    ****************************************/

    // Black Screen
    if (displayNumber == 0){                        // Black Screen
        tft.fillScreen(TFT_BLACK);                  // Set screen all black
        digitalWrite(_pinBL, LOW);                  // Turn off backlight on display
    }

    // Lock Screen
    else if (displayNumber == 1){                   // Lock Screen
        
        
        
        tft.setTextSize(1);
        
        tft.drawString("Temp: "     , 5, 80, 2);
        tft.drawString("Hum: "      , 5, 100, 2);
        tft.drawString("Press: "    , 5, 120, 2);

        tft.drawString("Lat: "      , 5, 140, 2);
        tft.drawString("Long: "     , 5, 160, 2);
        tft.drawString("Alt: "      , 5, 180, 2);

    };

    // Reset compare for next scan
    min_prev = minute(); 
    displayNumber_prev = displayNumber;
    batteryPercent_prev = batteryPercent;


}