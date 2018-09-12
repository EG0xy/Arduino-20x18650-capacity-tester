// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       V3.ino
    Created:	2018-08-21 00:30:04
    Author:     DESKTOP-LRDJEN8\Hamer
*/

#include <SPI.h>
#include <Wire.h>

#include "Debug.h"

#include "Predefined.h"
#include "LCD.h"
#include "PWM_Drive.h"
#include "Battery.h"

Battery batteries[BAT_COUNT];

void setup()
{
	Wire.begin();
	Serial.begin(9600);
	
	setupMux();
	setupPWM_Drives();
	
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i] = Battery();
		batteries[i].init();
	}

}

// Add the main program code into the continuous loop() function
void loop()
{
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].updateVoltage(Voltage);
		batteries[i].updateVoltage(Load);
		batteries[i].UpdateCapacity();
	}

}
