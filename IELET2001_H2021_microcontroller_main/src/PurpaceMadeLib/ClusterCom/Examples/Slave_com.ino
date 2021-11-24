/*
 Name:    LibraryCreater.ino
 Created: 11/9/2021 3:56:20 PM
 Author:  Andreas Vik
 */

#include "ClusterCom.h"

// Address to Slave = 1-254
// Rx_pin = 26
// Tx_pin = 27
ClusterCom CCom(1);

// the setup function
void setup()
{
  CCom.begin();
}

// the loop function
void loop()
{
  if (CCom.available()) {
    // Print recived data (String)
    Serial.println(CCom.output());
  }
} 
