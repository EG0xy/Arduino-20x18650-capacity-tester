#pragma once
#define USE_SERIAL_INFO
//#define DISABLE_CHARGING
//#define DEBUG

#ifdef DEBUG
static int bt = 0;
static int pwmOffset = 0;
static bool bPWMTogle = false;
#endif // DEBUG

template<class T>
void SerialPrint(T value,bool newLine = true)
{
#ifdef USE_SERIAL_INFO
	Serial.print(value);
	if (newLine)
		Serial.println();
	else
		Serial.print(" ");
#endif
}
