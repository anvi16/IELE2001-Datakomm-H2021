/*
 Name:		LibraryCreater.ino
 Created:	11/9/2021 3:56:20 PM
 Author:	
 */

#include "ClusterCom.h"


ClusterCom CCom(0);

// the setup function
void setup()
{
	CCom.begin();
}

// the loop function
void loop()
{
	if (CCom.available()) {
		Serial.println(CCom.output());
	}

	CCom.send(1, "Hello World"); // Supports 31 Char
	delay(2000);
} 
