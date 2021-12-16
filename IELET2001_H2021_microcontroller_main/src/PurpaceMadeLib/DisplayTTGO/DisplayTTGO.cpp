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
    tft.setTextColor(TFT_WHITE);

    master = false;                 // Init
    ubi = false;                    // Init

    min_prev = 59;                  // Init
    batteryPercent = 50;            // Init
    batteryVoltage = 3.01;          // Init
    batteryPercent_prev = 0;        // Init
    batteryVoltage_prev = 0.0;      // Init

    lockScreenData[0] = 0.0;        // Temperature
    lockScreenData[1] = 0.0;        // Humidity
    lockScreenData[2] = 0.0;        // Pressure
    lockScreenData[3] = 0.0;        // Latitude
    lockScreenData[4] = 0.0;        // Longitude
    lockScreenData[5] = 0.0;        // Altitude

    lockScreenData_prev[0] = 0.0;    // Temperature
    lockScreenData_prev[1] = 0.0;    // Humidity
    lockScreenData_prev[2] = 0.0;    // Pressure
    lockScreenData_prev[3] = 0.0;    // Latitude
    lockScreenData_prev[4] = 0.0;    // Longitude
    lockScreenData_prev[5] = 0.0;    // Altitude

    
};

// Select displays
void DisplayTTGO::selectBlackScreen(){              // 0
    displayNumber = 0;
}
void DisplayTTGO::selectLockScreen(){               // 1
    displayNumber = 1;
}
void DisplayTTGO::selectMessageScreen(){            // 2
    displayNumber = 2;
}


void DisplayTTGO::refresh(){

    // Turn display backlight ON if BlackScreen has previously been displayed
    if ((displayNumber_prev == 0) && (displayNumber != 0)){
        digitalWrite(_pinBL, HIGH);
    }
    
    // Wipe display when togling between screens 
    if(displayNumber != displayNumber_prev){
        tft.fillRect(1,18,133,220,TFT_BLACK);
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

    // Show "Master" symbol
    if (master)drawMaster();
    
    // Show / clear "Ubidots" symbol
    if (ubi) drawUbi();
    else tft.fillRoundRect(60, 1, 12, 13, 1, TFT_BLACK);


    tft.drawRoundRect(110,4,24,10,1,TFT_WHITE);
    tft.fillRect(108,6,2,6,TFT_WHITE);

    /****************************************
    * Define different screen specs
    ****************************************/

    // Black Screen
    if (displayNumber == 0){                        // Black Screen
        digitalWrite(_pinBL, LOW);                  // Turn off backlight on display
    }

    // Lock Screen
    else if (displayNumber == 1){                   // Lock Screen
        
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(1);
        
        // WEATHER
        tft.drawCentreString("Weather",67, 30,  4);
        
        // Text
        tft.drawCentreString("T"    , 23, 60, 2);
        tft.drawCentreString("Hum"  , 67, 60, 2);
        tft.drawCentreString("P"    , 111,60, 2);

        // If no data is available, print "?"
        if((lockScreenData[0] == 0.0) && (lockScreenData[1] == 0.0) && (lockScreenData[2] == 0.0)){
            tft.fillRect(1,80,133,18,TFT_BLACK);                            // Clear relevant part of screen
            tft.drawCentreString("?", 23, 80, 2); // Print temperature to monitor
            tft.drawCentreString("?", 67, 80, 2); // Print humidity to monitor
            tft.drawCentreString("?", 111,80, 2); // Print pressure to monitor
        }
        // Redraw dynamic field if changes have been made
        else if (   (lockScreenData[0] != lockScreenData_prev[0]) ||                // If either temp, hum or press has ben updated 
                    (lockScreenData[1] != lockScreenData_prev[1]) ||
                    (lockScreenData[2] != lockScreenData_prev[2])){
            tft.fillRect(1,80,133,18,TFT_BLACK);                            // Clear relevant part of screen
            tft.drawCentreString(String(lockScreenData[0],1),   23, 80, 2); // Print temperature to monitor
            tft.drawCentreString(String(lockScreenData[1],1),   67, 80, 2); // Print humidity to monitor
            tft.drawCentreString(String(lockScreenData[2],1),   111,80, 2); // Print pressure to monitor
        }
        
        // Format latitude and longitude in a string
        char gpsData[30] = "";
        sprintf(gpsData, "%.3fN, %.3fE", lockScreenData[3], lockScreenData[4]);
        
        // POSITION
        tft.drawCentreString("Position", 67, 140,  4);
        
        // Coordinates
        tft.drawCentreString("Coordinates:", 67, 170, 1);
        if((lockScreenData[3] == 0.000) && (lockScreenData[4] == 0.000) && (displayNumber != displayNumber_prev)){ // If null island, print error
            tft.fillRect(1,180,133,18,TFT_BLACK);                           // Clear relevant part of screen
            tft.drawCentreString("No GPS data", 67, 180, 2);
        }
        else if ((lockScreenData[3] != lockScreenData_prev[3]) ||           // If either lat or lng has been updated
            (lockScreenData[4] != lockScreenData_prev[4])){
            tft.fillRect(1,180,133,18,TFT_BLACK);                           // Clear relevant part of screen
            tft.drawCentreString(gpsData, 67, 180, 2);                      // Print position on monitor
        }
        

        // Altitude
        tft.drawCentreString("Altitude [m.a.s]:", 67, 210, 1);
        if((lockScreenData[5] == 0.00000) && (displayNumber != displayNumber_prev)){    // If null values, print error 
            tft.fillRect(1,220,133,18,TFT_BLACK);                           // Clear relevant part of screen
            tft.drawCentreString("No GPS data", 67, 220, 2);
        }
        else if (lockScreenData[5] != lockScreenData_prev[5]){              // If alt has been updated
            tft.fillRect(1,220,133,18,TFT_BLACK);                           // Clear relevant part of screen
            tft.drawCentreString(String(lockScreenData[5],1), 67, 220, 2);  // Print altitude on monitor
        }


        // Comparator for next scan
        for (int i = 0; i<6; i++){
            lockScreenData_prev[i] = lockScreenData[i];
        }
    }
    
    // Message screen
    else if (displayNumber == 2){                               // Generic message screen
        drawMessageScreenStrings();
    }

    // Reset compare for next scan
    min_prev = minute(); 
    displayNumber_prev = displayNumber;
    batteryPercent_prev = batteryPercent;

}
void DisplayTTGO::setBatteryState(int percent, float volt){
    batteryPercent = percent;
    batteryVoltage = volt;
}
void DisplayTTGO::setLockScreenData(float t, float h, float p, float lat, float lng, float alt){
    
    // Temp, hum, press, lat, lng, alt

    lockScreenData[0] = t;                          // Temperature
    lockScreenData[1] = h;                          // Humidity
    lockScreenData[2] = p;                          // Pressure
    lockScreenData[3] = lat;                        // Latitude
    lockScreenData[4] = lng;                        // Longitude
    lockScreenData[5] = alt;                        // Altitude

}
void DisplayTTGO::setString(int nLine, String text){
    str[nLine] = text;
}
void DisplayTTGO::clearStrings(){
    for (int i = 0; i < 10; i++){
        str[i] = "";
    }
}
void DisplayTTGO::setMaster(){
    master = true;
}
void DisplayTTGO::setUbi(){
    ubi = true;
}
void DisplayTTGO::resetUbi(){
    ubi = false;
}


// Private
void DisplayTTGO::drawBatteryState(int percent){

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
void DisplayTTGO::drawMessageScreenStrings(){

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    
    if (str[0] != str_prev[0]){
        tft.fillRect(1, 30, 133, 25, TFT_BLACK);
        tft.drawCentreString(str[0],67, 30,  4);
    }

    if (str[1] != str_prev[1]){
        tft.fillRect(1, 60, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[1],67, 60,  2);
    }

    if (str[2] != str_prev[2]){
        tft.fillRect(1, 80, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[2],67, 80,  2);
    }

    if (str[3] != str_prev[3]){
        tft.fillRect(1, 100, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[3],67, 100,  2);
    }

    if (str[4] != str_prev[4]){
        tft.fillRect(1, 120, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[4],67, 120,  2);
    }

    if (str[5] != str_prev[5]){
        tft.fillRect(1, 140, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[5],67, 140,  2);
    }

    if (str[6] != str_prev[6]){
        tft.fillRect(1, 160, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[6],67, 160,  2);
    }

    if (str[7] != str_prev[7]){
        tft.fillRect(1, 180, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[7],67, 180,  2);
    }

    if (str[8] != str_prev[8]){
        tft.fillRect(1, 200, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[8],67, 200,  2);
    }

    if (str[9] != str_prev[9]){
        tft.fillRect(1, 220, 133, 18, TFT_BLACK);
        tft.drawCentreString(str[9],67, 220,  2);
    }

    // Comparator for next scan
    for (int i = 0; i<10; i++){
        str_prev[i] = str[i];
    }
}
void DisplayTTGO::drawMaster(){
    
    tft.fillRoundRect(45, 1, 13, 13, 1, TFT_WHITE);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("M", 47, 0, 2);

}

void DisplayTTGO::drawUbi(){
    
    tft.fillRoundRect(60, 1, 12, 13, 1, TFT_WHITE);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("U", 62, 0, 2);

}