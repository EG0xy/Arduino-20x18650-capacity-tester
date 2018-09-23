#pragma once
#include "Battery.h"
#include <LiquidCrystal_I2C.h>

#define LCD_TIMEOUT 10000
#define LCD_REFRESH 500

#define LCD_ON -1
#define LCD_OFF -2

static LiquidCrystal_I2C lcd(0x3f, 16, 2);

short lastScreen = LCD_ON;
unsigned long lcdTimeOut = LCD_TIMEOUT;
unsigned long screenRefresh = LCD_REFRESH;
unsigned long batteryLastInfoTimer = 0;
bool bLastInfoTrigered = false;

void setupLCD()
{
	lcd.begin(16, 2);
	lcd.setBacklight(LOW);
};

void EnableLCD(float timeMultiplier = 1.f)
{
	lcd.setBacklight(LOW);
	lcdTimeOut = millis() + (unsigned long)(round((float)LCD_TIMEOUT*timeMultiplier));
};

void UpdateDisplay()
{
	if (screenRefresh >= millis()) return;

	if (lcdTimeOut > 0 && screenRefresh < millis())
	{
		screenRefresh = millis() + LCD_REFRESH;
	}

	if (lastScreen == LCD_ON) // MAIN screen
	{
		lastScreen = LCD_OFF; // To prevent clearing screen repeatedly
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print("Battery Tester");
		lcd.setCursor(5, 1);
		lcd.print("Ver 2");
		EnableLCD();
	}
	else if (lastScreen >= 0 && lcdTimeOut > millis()) // Display the last active battery for some time then reset to MAIN screen
	{
		switch (batteries[lastScreen].state)
		{
		case State::Empty:
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			lcd.print("Batter ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(1, 1);
			lcd.print("Empty  Slot!");
			break;
		}
		case State::Low:
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			lcd.print("Battery ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(3, 1);
			lcd.print("LOW: ");
			lcd.setCursor(8, 1);
			lcd.print(batteries[lastScreen].voltage);
			lcd.setCursor(12, 1);
			lcd.print("V");
			break;
		}
		case State::Charging:
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			lcd.print("Battery ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(1, 1);
			lcd.print("Charging");
			lcd.setCursor(10, 1);
			lcd.print(batteries[lastScreen].voltage);
			lcd.setCursor(14, 1);
			lcd.print("V");
			break;
		}case State::Discharging:
		{
			lcd.clear();
			lcd.setCursor(1, 0);
			lcd.print("Bat-");
			lcd.setCursor(5, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(8, 0);
			lcd.print(round(batteries[lastScreen].capacity));
			lcd.setCursor(12, 0);
			lcd.print("mAh");
			lcd.setCursor(0, 1);
			lcd.print(batteries[lastScreen].voltage);
			lcd.setCursor(4, 1);
			lcd.print("V");
			lcd.setCursor(6, 1);
			lcd.print(round(batteries[lastScreen].resistanceData.resistance));
			lcd.setCursor(9, 1);
			lcd.print("mO");
			//lcd.setCursor(10, 1);
			//lcd.write(0);
			lcd.setCursor(12, 1);
			if ((lcdTimeOut - LCD_TIMEOUT / 2) > millis())
			{
				lcd.print(batteries[lastScreen].loadVoltage);
				lcd.setCursor(15, 1);
				lcd.print("A");
			}
			else
			{
				lcd.print(round(batteries[lastScreen].temperature));
				lcd.setCursor(14, 1);
				lcd.print("C");
				//lcd.write(1);
			}
			break;
		}case State::Recharging:
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			lcd.print("Battery ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(0, 1);
			lcd.print("ReCharging");
			lcd.setCursor(11, 1);
			lcd.print(batteries[lastScreen].voltage);
			lcd.setCursor(15, 1);
			lcd.print("V");
			break;
		}case State::Finished:
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			lcd.print("Batterie ");
			lcd.setCursor(12, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(2, 1);
			lcd.print(round(batteries[lastScreen].capacity));
			lcd.setCursor(6, 1);
			lcd.print("mAh");
			lcd.setCursor(10, 1);
			lcd.print(round(batteries[lastScreen].resistanceData.resistance));
			lcd.setCursor(13, 1);
			lcd.print("mO");
			//lcd.setCursor(14, 1);
			//lcd.write(0);
			break;
		}
		default:
			break;
		}
	}
	else if (lcdTimeOut <= millis() && lcdTimeOut > 0)
	{
		lastScreen = LCD_OFF;
		lcdTimeOut = 0;
		lcd.clear();
		lcd.setBacklight(HIGH);
	}
};
