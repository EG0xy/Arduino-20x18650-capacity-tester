#pragma once

#define INPUT_PIN A0

enum class EKeyID :int
{
	None = -1,
	Right = 0,
	Left,
	Up,
	Down,
	Select
};

EKeyID activeButton = EKeyID::None;
bool InputActive = false;

void setupInput()
{
	//buttons
	pinMode(A0, INPUT);
};

void ResetLastInfo()
{
	if (bLastInfoTrigered)
	{
		bLastInfoTrigered = false;
		if (lastScreen > LCD_ON)
			batteries[lastScreen].Reset();
		batteryLastInfoTimer = millis();
	}
};

int PrioritySelect(int step, int nLast, bool bNoFilter = false)
{
	int result = max(nLast, 0);

	if (nLast >= 0)
	{
		ResetLastInfo();
		result = result + step;
	}

	if (bNoFilter)
	{
		if (result < 0)
		{
			result = BAT_COUNT - 1;
		}
		else if (result >= BAT_COUNT)
		{
			result = 0;
		}
		return result;
	}

	int nCount = 0;
	bool bFound = false;
	State searchState = State::Finished;
#ifdef DEBUG
	Serial.print("Entering Priority Selecttion loop");
	Serial.println();
#endif
	do {
		nCount += 1;

		//checking search bounds
		if (result < 0)
		{
			result = BAT_COUNT - 1;
		}
		else if (result >= BAT_COUNT)
		{
			result = 0;
		}

		if (batteries[result].state == searchState)
		{
			bFound = true;
		}
		else
		{
			result = result + step;
		}

		if (!bFound && nCount == BAT_COUNT && searchState > State::Low)
		{
			searchState = (State)((int)searchState - 1);
#ifdef DEBUG
			Serial.print("Priority Selecttion loop: switch to ");
			Serial.print((int)searchState);
			Serial.print(" state");
			Serial.println();
#endif
			nCount = 0;
		}

	} while (!bFound && nCount <= BAT_COUNT);

	if (!bFound)
		result = LCD_ON;

#ifdef DEBUG
	Serial.print("Priority Selecttion loop: returning ");
	Serial.print(result);
	Serial.println();
#endif

	return result;
};

void UpdateInput()
{
	//Disable controlls for when battery is removed
	if (bLastInfoTrigered && lastScreen > LCD_ON) return;

	int readkey = analogRead(0);

	if (activeButton == EKeyID::None)
	{
		if (readkey < 30) {
			activeButton = EKeyID::Right;
		}
		if (readkey >= 30 && readkey < 160) {
			activeButton = EKeyID::Up;
		}
		if (readkey >= 160 && readkey < 400) {
			activeButton = EKeyID::Down;
		}
		if (readkey >= 400 && readkey < 550) {
			activeButton = EKeyID::Left;
		}
		if (readkey >= 550 && readkey < 800) {
			activeButton = EKeyID::Select;
		}
	}
	else if (readkey >= 740) {
		InputActive = false;
		activeButton = EKeyID::None;
#ifdef DEBUG
		Serial.print("All released");
		Serial.println();
#endif
	}

	if (activeButton != EKeyID::None)
	{
		EnableLCD();

		//if (lastScreen <= LCD_ON)
		//{
		//	InputActive = true;
		//	lastScreen = PrioritySelect(1, lastScreen);
		//	return;
		//}
	}

	if (!InputActive)
	{
		switch (activeButton)
		{
		case EKeyID::Right:
		{
			InputActive = true;


#ifdef DEBUG
			Serial.print("RIGHT");
			Serial.println();
			pwmOffset += 50;

			if (PWM_OFF + pwmOffset > MAX_PWM)
				pwmOffset = MAX_PWM - PWM_OFF;
#endif
			break;
		}
		case EKeyID::Left:
		{
			InputActive = true;
#ifdef DEBUG
			Serial.print("LEFT");
			Serial.println();
			pwmOffset -= 50;

			if (PWM_OFF + pwmOffset < 0)
				pwmOffset = -PWM_OFF;
#endif
			break;
		}
		case EKeyID::Up:
		{
			InputActive = true;
#ifdef DEBUG
			Serial.print("UP");
			Serial.println();

			bt += 1;

			if (bt >= BAT_COUNT)
				bt = 0;
#else        
			EnableLCD();//refreshing timer
			lastScreen = PrioritySelect(1, lastScreen, nFinishedCount == 0);
#endif
			break;
		}
		case EKeyID::Down:
		{
			InputActive = true;
#ifdef DEBUG
			Serial.print("DOWN");
			Serial.println();

			bt -= 1;

			if (bt < 0)
				bt = BAT_COUNT - 1;
#else 
			EnableLCD();//refreshing timer
			lastScreen = PrioritySelect(-1, lastScreen, nFinishedCount == 0);
#endif
			break;
		}
		case EKeyID::Select:
		{
			InputActive = true;
#ifdef DEBUG
			Serial.print("SELECT");
			Serial.println();

			if (!bPWMTogle)
			{
				pwmOffset = -PWM_OFF;
				bPWMTogle = true;
			}
			else
			{
				pwmOffset = 0;
				bPWMTogle = false;
			}

#endif
			break;
		}
		default:
		{
		}

		};
	}
};
