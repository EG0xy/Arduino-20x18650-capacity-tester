// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       V3.ino
    Created:	2018-08-21 00:30:04
    Author:     DESKTOP-LRDJEN8\Hamer
*/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Debug.h"
#include "Battery.h"
#include "Predefined.h"
#include "LCD.h"
#include "PWM_Drive.h"

#include "Input.h"

Battery batteries[BAT_COUNT];



void setup()
{
	Wire.begin();
	Serial.begin(9600);
	
	setupPWM_Drives();
	setupMux();
	
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i] = Battery();
		batteries[i].init(voltageReadCalibration[i]);
	}

}

// Add the main program code into the continuous loop() function
void loop()
{
	SerialPrint("**********************************************************");
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].updateVoltage(Voltage);
		SerialPrint(batteries[i].voltage,false);
		batteries[i].updateVoltage(Load);
		SerialPrint(batteries[i].loadVoltage, false);
		batteries[i].updateTemp();
		SerialPrint(batteries[i].temperature, false);
		batteries[i].UpdateCapacity();
		SerialPrint(batteries[i].pwm, false);

		SerialPrint(" ");
	}

#ifndef DEBUG
	UpdateState();
#endif // !DEBUG
	UpdateInput();
#ifndef DEBUG
	UpdateDisplay();
#endif // !DEBUG

#ifdef DEBUG

	for (int i = 0; i < BAT_COUNT; ++i)
	{
		if (batteries[i].pwm != ((int)PWM_OFF + pwmOffset))
		{
			batteries[i].pwm = ((int)PWM_OFF + pwmOffset);
      setPWM(batteries[i], -1.f);
			//SetPWM(batteries[i].loadVoltage * 1000.f, i);
		}
	}
	//lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("L: ");
	lcd.setCursor(3, 0);
	lcd.print(batteries[bt].loadVoltage);
	lcd.setCursor(7, 0);
	lcd.print("V");
	lcd.setCursor(9, 0);
	lcd.print(batteries[bt].temperature);
	lcd.setCursor(14, 0);
	lcd.print((char)0xDF);
	lcd.print("C");

	lcd.setCursor(0, 1);
	lcd.print("    ");
	lcd.setCursor(0, 1);
	lcd.print(batteries[bt].pwm);
	lcd.setCursor(5, 1);
	lcd.print(batteries[bt].voltage);
	lcd.setCursor(10, 1);
	lcd.print("V");

	lcd.setCursor(12, 1);
	lcd.print("BT");
	lcd.setCursor(15, 1);
	lcd.print(" ");
	lcd.setCursor(14, 1);
	lcd.print(bt + 1);

#endif // DEBUG

}

void UpdateState()
{
	nFinishedCount = 0;
	for (size_t i = 0; i < BAT_COUNT; ++i)
	{
		bool reinserted = bLastInfoTrigered &&
			lastScreen == i &&
			batteries[i].state != State::Finished &&
			batteries[i].state != State::Charging &&
			batteryLastInfoTimer > millis() &&
			batteries[lastScreen].voltage > m_cfBatteryCutOffVoltage;

		if (reinserted)
		{
			bLastInfoTrigered = false;
			batteryLastInfoTimer = millis() + LCD_TIMEOUT / 2;
		}

		if (!reinserted && batteries[i].state != State::Empty && batteries[i].voltage <= m_cfBatteryCutOffVoltage - 0.5f)
		{
			if (!bLastInfoTrigered)
			{
				EnableLCD();
				bLastInfoTrigered = true;
				batteryLastInfoTimer = millis() + LCD_TIMEOUT;
				lastScreen = i;
			}
			else
			{
				if (lastScreen == i)
				{
					if (batteryLastInfoTimer <= millis())
					{
						batteries[lastScreen].Reset();
						bLastInfoTrigered = false;
						batteryLastInfoTimer = millis();
						lastScreen = PrioritySelect(1, lastScreen);

						if (lastScreen >= 0)
						{
							batteryLastInfoTimer = millis() + LCD_TIMEOUT / 2;
							EnableLCD(0.5f);
						}
					}
				}
				else
				{
					if (lastScreen >= 0)
						batteries[lastScreen].Reset();
					batteryLastInfoTimer = millis() + LCD_TIMEOUT;
					lastScreen = i;
					EnableLCD();
				}
			}
			continue;
		}

		switch (batteries[i].state)
		{
		case State::Empty:
		{

#ifndef DISABLE_CHARGING
			if (batteries[i].voltage >= m_cfBatteryCutOffVoltage) {
				batteries[i].state = State::Charging;

				//using this to delay the charge switching for correct voltage readings of TP 4056
				batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;


				if (!bLastInfoTrigered)
				{
					EnableLCD();
					lastScreen = i;
				}
			}
			else
#else
			if (batteries[i].voltage < m_cfBatteryDischargeVoltage)
			{
				batteries[i].state = State::Low;
			}
			else
#endif
				if (batteries[i].voltage >= m_cfBatteryDischargeVoltage)
				{
					batteries[i].state = State::Resistance;
					//break;
				}
				else
				{
					break;
				}
		}
		case State::Low:
		{
			if (batteries[i].state == State::Low)
			{
				if (batteries[i].voltage >= m_cfBatteryDischargeVoltage)// if you add another battery in to holder
				{
					batteries[i].state = State::Resistance;
					EnableLCD();
					lastScreen = i;
				}
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" LOW: ");
				Serial.print(batteries[i].voltage);
				Serial.print(" V");
				Serial.print(" Load: ");
				Serial.print(batteries[i].loadVoltage);
				Serial.print(" V");
				Serial.print(" PWM: ");
				Serial.print(batteries[i].pwm);
				Serial.println();
#endif
				break;
			}
		}
		case State::Charging:
		{
#ifdef USE_SERIAL
			Serial.print("Batterie : ");
			Serial.print(i + 1);
			Serial.print(" CHARGING");
			Serial.println();
#endif
#ifndef DISABLE_CHARGING

			if (batteries[i].state == State::Charging)
			{
				if (batteries[i].resistanceData.timer > millis())
					break;

				batteries[i].resistanceData.timer = 0;

				if (batteries[i].pwm != PWM_CHARGE)
				{
					batteries[i].pwm = PWM_CHARGE;
					setPWM(batteries[i], -1.f);
					break;
				}
				else if (batteries[i].temperature < 0.f) // ~-50 will be set when TP4056 STDBY activates
				{
					batteries[i].pwm = PWM_OFF;
					setPWM(batteries[i], -1.f);
					batteries[i].state = State::Resistance;
				}
				else
				{
					break;
				}
			}
#else
			batteries[i].state = State::Resistance;
#endif
		}
		case State::Resistance:
		{
			if (batteries[i].resistanceData.timer <= 0)
			{
				batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;

				if (!bLastInfoTrigered)
				{
					EnableLCD();
					lastScreen = i;
				}

				setPWM(batteries[i], PWM_I_1);
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" INIT resistance timer");
				Serial.print(" --- VCC : ");
				Serial.print(readVcc() * 0.001f);
				Serial.println();
#endif
			}
			else if (batteries[i].resistanceData.resistance == 0.0 && batteries[i].resistanceData.timer <= millis())
			{
				batteries[i].resistanceData.voltage[batteries[i].resistanceData.mesurementCount] = batteries[i].voltage;
				batteries[i].resistanceData.current[batteries[i].resistanceData.mesurementCount] = batteries[i].loadVoltage;

				++batteries[i].resistanceData.mesurementCount;

				if (batteries[i].resistanceData.mesurementCount >= 2)
				{
					//IEC 61960-2003
					batteries[i].resistanceData.resistance = 1000.f * fabsf(batteries[i].resistanceData.voltage[1] - batteries[i].resistanceData.voltage[0]) / fabsf(batteries[i].resistanceData.current[1] - batteries[i].resistanceData.current[0]);

#ifdef USE_SERIAL
					Serial.print("Batterie : ");
					Serial.print(i + 1);
					Serial.print(" resistance : ");
					Serial.print(batteries[i].resistance);
					Serial.print(" mOhm (IEC 61960-2003)");
					Serial.println();
					Serial.print("  V-1 : ");
					Serial.print(batteries[i].resistanceData.voltages[0]);
					Serial.print("  I-1 : ");
					Serial.print(batteries[i].resistanceData.currents[0]);
					Serial.println();
					Serial.print("  V-2 : ");
					Serial.print(batteries[i].resistanceData.voltages[1]);
					Serial.print("  I-2 : ");
					Serial.print(batteries[i].resistanceData.currents[1]);
					Serial.println();
#endif
				}
				else
				{
					batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;
					setPWM(batteries[i], PWM_I_2);
				}
			}

			batteries[i].state = State::Discharging;
		}
		case State::Discharging:
		{
			if (batteries[i].voltage <= m_cfBatteryCutOffVoltage)// && (lastScreen == i?!bLastInfoTrigered:true))
			{
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" DISCHARGE completed");
				Serial.println();
#endif
				batteries[i].pwm = PWM_OFF;
				setPWM(batteries[i], -1.f);
				batteries[i].state = State::Recharging;
#ifdef DISABLE_CHARGING
				batteries[i].voltage = m_cfRechargeVoltage;
#endif
			}
			else
			{
				if (batteries[i].pwm == PWM_CHARGE || batteries[i].pwm == PWM_OFF)
				{
					batteries[i].pwm = PWM_ON;
					batteries[i].lastTime = millis();
					if (batteries[i].resistanceData.mesurementCount >= 2)//Do not change PWM while resistance test running
						setPWM(batteries[i], -1.f);
					batteries[i].state = State::Discharging;
					break;
				}

				if (batteries[i].resistanceData.mesurementCount < 2 && batteries[i].resistanceData.timer <= millis())
				{
					batteries[i].state = State::Resistance;
				}

				batteries[i].UpdateCapacity();
				//UpdateCapacity(batteries[i].loadVoltage * 1000.f, i);
				if (batteries[i].resistanceData.mesurementCount >= 2)//Do not change PWM while resistance test running
					batteries[i].adjustPWM();
					//SetPWM(batteries[i].loadVoltage * 1000.f, i);
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" Capacity: ");
				Serial.print(batteries[i].capacity);
				Serial.print(" --- V : ");
				Serial.print(batteries[i].voltage*1000.f);
				Serial.print(" --- A : ");
				Serial.print(batteries[i].loadVoltage*1000.f);
				Serial.print(" --- PWM : ");
				Serial.print(batteries[i].pwm);
				Serial.println();
#endif
				break;
			}
		}
		case State::Recharging:
		{
#ifdef USE_RECHARGE
			if (batteries[i].voltage >= m_cfRechargeVoltage)
			{
				batteries[i].pwm = PWM_OFF;
				setPWM(batteries[i], -1.f);
				batteries[i].state = State::Finished;
			}
			else
			{

				if (batteries[i].pwm == PWM_OFF)
				{
					batteries[i].pwm = PWM_CHARGE;
					setPWM(batteries[i], -1.f);
				}

#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" RECHARGING");
				Serial.println();
#endif
				break;
			}
#else
			if (!bLastInfoTrigered)// && lastScreen <= LCD_ON)//Finished screen is shown
			{
				EnableLCD();
				lastScreen = i;
			}
			batteries[i].state = State::Finished;
#endif
		}
		case State::Finished:
		{
#ifdef USE_SERIAL
			Serial.print("Batterie : ");
			Serial.print(i + 1);
			Serial.print(" FINISHED");
			Serial.println();
#endif
		}
		default:
			break;
		}

		if (batteries[i].state == State::Finished)
			++nFinishedCount;
	}
}
