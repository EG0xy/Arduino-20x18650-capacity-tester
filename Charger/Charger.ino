//#include <TimerOne.h>
//#include <Servo.h>
#include <SPI.h>
#include <ShiftPWM.h>
#include <LiquidCrystal_I2C.h>
#include <CD74HC4067.h>
#include <Wire.h>
/* Connection
   A0 - LCD Keypad shield - 5 keys array
        ~0 - Right
        ~130 - Up 
        ~306 - Down
        ~479 - Left
        ~719 - Select

   A1 - Multiplexer HC4067 (SIG pin) Battery (16) + Voltage Reading
   A2 - Batery (17) + Voltage Reading
   A3 - Batery (18) + Voltage Reading
   A4 - Batery (19) + Voltage Reading
   A5 - Batery (20) + Voltage Reading

   D42 - Multiplexer HC4067 (S0 pin) Read control
   D44 - Multiplexer HC4067 (S1 pin) Read control
   D46 - Multiplexer HC4067 (S2 pin) Read control
   D48 - Multiplexer HC4067 (S3 pin) Read control
   D45 - Shift Register 74HC595N X 3 (ST_CP pin)
   D51 - Shift Register 74HC595N X 3 (DS pin MOSI)
   D52 - Shift Register 74HC595N X 3 (SH_CP pin)

   81.85931902590525 - 0.11406781021018966x + 0.00007701200364055409x2 - 2.666253770521e-8x3 + 3.54777456e-12x4 - formula for % diff from OPUS
*/

//#define DEBUG
#define SCREEN_REFRESH 500
#define SCREEN_TIMEOUT 7000
#define MAIN_SCREEN -1
#define SCREEN_OFF -2

#define BAT_COUNT 20
#define BAT_READ_COUNT 10
const float m_cfBatteryDischargeVoltage = 4.1;
const float m_cfBatteryCutOffVoltage = 2.85;

#define MUX_CH_COUNT 16
#define MUX_S0 42
#define MUX_S1 44
#define MUX_S2 46
#define MUX_S3 48
#define MUX_SIG A1
//Aditional Analog pins for last 4 batterys (MUX has only 16)
#define BAT_17 A2
#define BAT_18 A3
#define BAT_19 A4
#define BAT_20 A5

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address for a 16 chars and 2 line display
CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3); //Multiplexer control

enum class EKeyID:int
{
	None = -1,
  	Right = 0,
  	Left,
  	Up,
  	Down,
  	Select
};

enum class EState:int
{
  Empty = 0,
  Charging,
  Discharging,
  Finished,
  Low
};

struct Battery
{
	EState m_State;
	float m_Capacity;
	float m_Voltage;
	float m_Resistance;
	unsigned long m_Time;
	unsigned long m_LastTime;
	bool m_ResetPWM;

	Battery(): m_State(EState::Empty), m_Capacity(0.0), m_Voltage(0.0)/*, m_Current(1000.0)*/, m_Time(0), m_LastTime(millis()),m_ResetPWM(false){};  
};

Battery m_sBatterys[BAT_COUNT];

int batPins[] = {BAT_17, BAT_18, BAT_19, BAT_20};

/*HC595*/
const int ShiftPWM_latchPin = 45;
const bool ShiftPWM_invertOutputs = 0;
unsigned char maxBrightness = 100;
unsigned char pwmFrequency = 200; //DO NOT CHANGE pwmFrequency after calibrating pwmDuty and voltageCalibration
const int numRegisters = 3;

short lastScreen = MAIN_SCREEN;
unsigned long screenTimeOut = SCREEN_TIMEOUT;
unsigned long screenRefresh = SCREEN_REFRESH;
unsigned long batteryLastInfoTimer = 0;
bool bLastInfoTrigered = false;

EKeyID activeButton = EKeyID::None;
bool InputActive = false;


//mesured to draw 1000 mA(in reality is 1.013-1.050)
const unsigned short pwmDuty[8 * numRegisters] = {
  21,//1
  21,//2
  19,//3
  19,//4
  19,//5
  19,//6
  19,//7
  19,//8
  19,//9
  19,//10
  19,//11
  19,//12
  19,//13
  19,//14
  19,//15
  19,//16
  19,//17
  19,//18
  19,//19
  19,//20 10 = 0.520A 37 = 1.524A
  0,  //21 -- not used
  0,  //22 -- not used
  0,  //23 -- not used
  0   //24 -- not used
/*
  0.2296,//1
  0.2147,//2
  0.2024,//3
  0.2115,//4
  0.1971,//5
  0.2042,//6
  0.2060,//7
  0.2088,//8
  0.1995,//9
  0.1966,//10
  0.1912,//11
  0.1961,//12
  0.1951,//13
  0.1923,//14
  0.1990,//15
  0.1992,//16
  0.2044,//17
  0.1953,//18
  0.2082,//19
  0.1900,//20
  0.0,  //21 -- not used
  0.0,  //22 -- not used
  0.0,  //23 -- not used
  0.0   //24 -- not used*/
};

//readings calibrated by my multimeter
float voltageCalibration[BAT_COUNT]=
{
    0.9957,//1
    0.9999,//2
    1.0007,//3
    1.0053,//4
    0.9893,//5
    0.9962,//6
    0.9937,//7
    0.9965,//8
    0.9994,//9
    0.9993,//10
    0.9943,//11
    0.9952,//12
    0.9952,//13
    1.0070,//14
    1.0083,//15
    1.0000,//16
    0.9994,//17
    0.9913,//18
    0.9943,//19
    0.9898//20
};

//readings calibrated by my multimeter
float currentCalibration[BAT_COUNT]=
{
    1023.0,//1
    1028.0,//2
    1026.0,//3
    1021.0,//4
    1028.0,//5
    1031.0,//6
    1027.0,//7
    1028.0,//8
    1026.0,//9
    1029.0,//10
    1029.0,//11
    1029.0,//12
    1030.0,//13
    1032.0,//14
    1030.0,//15
    1030.0,//16
    1025.0,//17
    1029.0,//18
    1021.0,//19
    1025.0//20
};

float getVoltage(uint8_t pin) {                     //read voltage from analog pins
  float sample = 0.0;
  for (uint8_t i = 0; i < BAT_READ_COUNT; i++) {
    sample += analogRead(pin);
    delay(1);
  }
  sample = sample / BAT_READ_COUNT;
  return (float)sample * 2 * (5.0 / 1023.0);
}

void UpdateBatteryVoltage(int nBat)// index from 1
{
  float fVolts = 0.f;
  switch (nBat) {
    case 16:
    case 17:
    case 18:
    case 19:
      {
        fVolts = getVoltage(batPins[nBat - 16]);
        break;
      }
    default:
      {
        mux.channel(nBat);
        delay(1);
        fVolts = getVoltage(MUX_SIG);
      }
  }
  m_sBatterys[nBat].m_Voltage = fVolts * voltageCalibration[nBat];
}

void UpdateCapacity(int nBat)
{
    unsigned long nDeltaTime = millis() - m_sBatterys[nBat].m_LastTime;
    m_sBatterys[nBat].m_Capacity += (currentCalibration[nBat] * (float)nDeltaTime) / 3600000.0; //A*deltaTime/1hour
    m_sBatterys[nBat].m_LastTime = millis();

    m_sBatterys[nBat].m_Time += nDeltaTime;
}

void ResetLastInfo()
{
  if(bLastInfoTrigered)
  {
    bLastInfoTrigered = false;
    m_sBatterys[lastScreen].m_State = EState::Empty;
    batteryLastInfoTimer = millis();
  }
}

void UpdateState(int nBat)
{
  UpdateBatteryVoltage(nBat);

  bool reinsertedAfterRemove = bLastInfoTrigered && lastScreen == nBat && batteryLastInfoTimer <= millis() && m_sBatterys[nBat].m_Voltage > m_cfBatteryCutOffVoltage;

  if(reinsertedAfterRemove)
  {
    bLastInfoTrigered = false;
    batteryLastInfoTimer = millis();
  }

  if(!reinsertedAfterRemove && m_sBatterys[nBat].m_State != EState::Empty && m_sBatterys[nBat].m_Voltage < m_cfBatteryCutOffVoltage - 1.0)
  {
    if(!bLastInfoTrigered)
    {
      bLastInfoTrigered = true;
      batteryLastInfoTimer = millis() + SCREEN_TIMEOUT;
      lastScreen = nBat;
      EnableLCD();
    }else{
      if(lastScreen == nBat)
      {
        if( batteryLastInfoTimer <= millis())
        {
          bLastInfoTrigered = false;
          m_sBatterys[lastScreen].m_State = EState::Empty;

          lastScreen = MAIN_SCREEN;
        }
      }else{
        if(lastScreen >= 0)//for crach prevention
          m_sBatterys[lastScreen].m_State = EState::Empty;
        batteryLastInfoTimer = millis() + SCREEN_TIMEOUT;
        lastScreen = nBat;
      }
    }
  }else if(m_sBatterys[nBat].m_State == EState::Discharging && m_sBatterys[nBat].m_Voltage <= m_cfBatteryCutOffVoltage)
  {
#ifdef DEBUG
    Serial.print("Finished SET");
    Serial.println();
#endif
    m_sBatterys[nBat].m_State = EState::Finished;
	m_sBatterys[nBat].m_ResetPWM = true;

    ResetLastInfo();
    lastScreen = nBat;
    EnableLCD();

  }else if (lastScreen == MAIN_SCREEN && (m_sBatterys[nBat].m_State == EState::Finished || m_sBatterys[nBat].m_State == EState::Low)) //When battery is pulled out from some slot, next Finished or Low will be searched for.
  {
    lastScreen = nBat;
    EnableLCD();    
  }else if(m_sBatterys[nBat].m_State == EState::Empty)
  {
    if(m_sBatterys[nBat].m_Voltage >= m_cfBatteryDischargeVoltage )
    {
#ifdef DEBUG
    Serial.print("Discharge SET");
    Serial.println();
#endif
      ResetLastInfo();
      
      lastScreen = nBat;
      m_sBatterys[nBat].m_State = EState::Discharging;
      m_sBatterys[nBat].m_LastTime = millis();
      m_sBatterys[nBat].m_Capacity = 0.0;
      m_sBatterys[nBat].m_Time = 0;
      m_sBatterys[nBat].m_ResetPWM = true;
      EnableLCD();
    }else if(m_sBatterys[nBat].m_Voltage > m_cfBatteryCutOffVoltage)
    {
#ifdef DEBUG
    Serial.print("Low SET");
    Serial.println();
#endif
      ResetLastInfo();

      lastScreen = nBat; // to display LOW if not charged battery added, and do see its voltages when adding

      m_sBatterys[nBat].m_State = EState::Low;
      EnableLCD();
    }
  }
}

void EnableLCD()
{
  lcd.setBacklight(LOW);
  screenTimeOut = millis() + SCREEN_TIMEOUT;
}

int PrioritySelect(int step,int nLast)
{
	int result = nLast;
	if(result < 0)
  {
		result = 0;
  }else
  {
    ResetLastInfo();
  }

	int nCount = 0;
	bool bFound = false;
	EState searchState = EState::Finished;

#ifdef DEBUG
    Serial.print("Entering Priority Selecttion loop");
    Serial.println();
#endif
	do{
		nCount += 1;

		result = result + step;

		//checking search bounds
		if (result < 0)
		{
			result = BAT_COUNT - 1;
		}else if ( result >= BAT_COUNT)
		{
			result = 0;
		}

		if(m_sBatterys[result].m_State == searchState)
		{
			bFound = true;
		}

		if(!bFound && nCount == BAT_COUNT && searchState == EState::Finished)
		{
#ifdef DEBUG
    Serial.print("Priority Selecttion loop: switch to Discharging");
    Serial.println();
#endif
			searchState = EState::Discharging;
			nCount = 0;
		}

	}while(!bFound && nCount <= BAT_COUNT);

  if(!bFound)
    result = MAIN_SCREEN;

#ifdef DEBUG
    Serial.print("Priority Selecttion loop: returning ");
    Serial.print(result);
    Serial.println();
#endif

	return result;
}

void UpdateInput()
{
	int readkey = analogRead(0);

#ifdef DEBUG
    Serial.print("Read Value: ");
    Serial.print(readkey);
    Serial.println();
#endif

	if(activeButton == EKeyID::None)
	{
		// if(readkey<30) {
		// 	activeButton = EKeyID::Right;
		// }
		if(readkey>= 30 && readkey<160) {
			activeButton = EKeyID::Up;
		}
		if(readkey>= 160 && readkey<330) {
			activeButton = EKeyID::Down;
		}
		// if(readkey>= 330 && readkey<500) {
		// 	activeButton = EKeyID::Left;
		// }
		// if(readkey>= 500 && readkey<740) {
		// 	activeButton = EKeyID::Select;
		// }

	}else if(readkey>= 740) {
      InputActive = false;
			activeButton = EKeyID::None;
#ifdef DEBUG
    Serial.print("All released");
    Serial.println();
#endif
	}

	if(activeButton != EKeyID::None)
	{
		EnableLCD();
		if(lastScreen < 0)
		{
      InputActive = true;
			lastScreen = PrioritySelect(1,lastScreen);
			return;
		}
	}

  	if(!InputActive)
  	{
		switch(activeButton)
		{
			case EKeyID::Right:
			{
    			InputActive = true;
#ifdef DEBUG
    Serial.print("RIGHT");
    Serial.println();
#endif
				break;
			}
			case EKeyID::Left:
			{
      			InputActive = true;
#ifdef DEBUG
    Serial.print("LEFT");
    Serial.println();
#endif
				break;
			}
			case EKeyID::Up:
			{
      			InputActive = true;
#ifdef DEBUG
    Serial.print("UP");
    Serial.println();
#endif        
				lastScreen = PrioritySelect(1,lastScreen);
				break;
			}
			case EKeyID::Down:
			{
      			InputActive = true;
#ifdef DEBUG
    Serial.print("DOWN");
    Serial.println();
#endif
				lastScreen = PrioritySelect(-1,lastScreen);
				break;
			}
			case EKeyID::Select:
			{
      			InputActive = true;
#ifdef DEBUG
    Serial.print("SELECT");
    Serial.println();
#endif
				break;
			}
			default:
			{
			}
		
		};
  	}

}

void UpdateDisplay()
{
  if(lastScreen == MAIN_SCREEN) // MAIN screen
  {
    lastScreen = SCREEN_OFF; // To prevent clearing screen repeatedly
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Battery Tester");
    lcd.setBacklight(LOW);
#ifdef DEBUG
    Serial.print("main screen switch to off");
    Serial.println();
#endif
  }else if(lastScreen >= 0 && screenTimeOut > millis()) // Display the last active battery for some time then reset to MAIN screen
  {
    if(m_sBatterys[lastScreen].m_State == EState::Low)
    {
#ifdef DEBUG
      Serial.print("low switch");
      Serial.println();
#endif    
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Battery ");
      lcd.setCursor(11, 0);
      lcd.print(lastScreen + 1);
      lcd.setCursor(3, 1);
      lcd.print("LOW: ");
      lcd.setCursor(8, 1);
      lcd.print(m_sBatterys[lastScreen].m_Voltage);
      lcd.setCursor(12, 1);
      lcd.print("V");
    }else if(m_sBatterys[lastScreen].m_State == EState::Finished)
    {
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Battery ");
      lcd.setCursor(11, 0);
      lcd.print(lastScreen + 1);
      lcd.setCursor(4, 1);
      lcd.print(round(m_sBatterys[lastScreen].m_Capacity));
      lcd.setCursor(9, 1);
      lcd.print("mAh");
    }else if(m_sBatterys[lastScreen].m_State == EState::Discharging)
    {
      void EnableLCD();
#ifdef DEBUG
      Serial.print("discharge");
      Serial.println();
#endif  
      
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Bat-");
      lcd.setCursor(5, 0);
      lcd.print(lastScreen + 1);
      lcd.setCursor(8, 0);
      lcd.print(round(m_sBatterys[lastScreen].m_Capacity));
      lcd.setCursor(12, 0);
      lcd.print("mAh");
      lcd.setCursor(1, 1);
      lcd.print(m_sBatterys[lastScreen].m_Voltage);
      lcd.setCursor(5, 1);
      lcd.print("V");
      lcd.setCursor(12, 1);
      size_t passedTime = m_sBatterys[lastScreen].m_Time / 1000;// convert to seconds
      size_t seconds = passedTime % 60;
      lcd.print(seconds);
      size_t hours = passedTime / 3600;
      size_t minutes = (passedTime - 3600 * hours) / 60;
      if(minutes > 0 || hours > 0)
      {
        lcd.setCursor(11, 1);
        lcd.print(":");
        lcd.setCursor(9, 1);
        lcd.print(minutes);
      }else
      {
        lcd.setCursor(14, 1);
        lcd.print("s");
      }
      if(hours > 0)
      {
        lcd.setCursor(8, 1);
        lcd.print(":");
        lcd.setCursor(7, 1);
        lcd.print(hours);
      }
    }else if(m_sBatterys[lastScreen].m_State == EState::Empty)
    {
    	EnableLCD();
#ifdef DEBUG
      	Serial.print("empty");
      	Serial.println();
#endif  
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Battery ");
      lcd.setCursor(11, 0);
      lcd.print(lastScreen + 1);
      lcd.setCursor(1, 1);
      lcd.print("Empty  Slot!");
    }
  }else if(screenTimeOut <= millis() && screenTimeOut > 0)
  {
#ifdef DEBUG
    Serial.print("time end");
    Serial.println();
#endif
    lastScreen = SCREEN_OFF;
    screenTimeOut = 0;
    lcd.clear();
    lcd.setBacklight(HIGH);
  }
}

void setup() {
  Wire.begin();

#ifdef DEBUG
  Serial.begin(9600);
#endif

  for (int i = 0; i < BAT_COUNT; ++i)
  {
    m_sBatterys[i] = Battery();
  }

  EnableLCD();

  lcd.begin(16, 2);
  UpdateDisplay();


  //*********************** Multiplexer HC4067 setup
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);

  pinMode(MUX_SIG, INPUT);
  //Adding aditional analog pins for 4 remaining battery voltage reads
  pinMode(BAT_17, INPUT);
  pinMode(BAT_18, INPUT);
  pinMode(BAT_19, INPUT);
  pinMode(BAT_20, INPUT);
  //buttons
  pinMode(A0, INPUT);
  //***********************************************
  pinMode(ShiftPWM_latchPin, OUTPUT);
  SPI.setBitOrder(LSBFIRST);
  // SPI_CLOCK_DIV2 is only a tiny bit faster in sending out the last byte.
  // SPI transfer and calculations overlap for the other bytes.
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  ShiftPWM.SetAmountOfRegisters(numRegisters);
  ShiftPWM.Start(pwmFrequency, maxBrightness);
  ShiftPWM.SetAll(0);
  delay(1);

  //  pinMode(13,OUTPUT);
  //  Timer1.initialize(10000);
  //  Timer1.attachInterrupt(timerIsr);
}

void loop() {

  // ShiftPWM.SetAll(0);
  for (int i = 0; i < min(BAT_COUNT,8 * numRegisters); ++i )
  {
	if(m_sBatterys[i].m_ResetPWM)
	{
		if(m_sBatterys[i].m_State == EState::Discharging)
		{
		  	ShiftPWM.SetOne(i, (unsigned char)pwmDuty[i]);
		}
		else
		{
			ShiftPWM.SetOne(i, 0);
		}
		m_sBatterys[i].m_ResetPWM = false;	
	}
  }
  delay(1);
  
  for (int i = 0; i < BAT_COUNT; ++i )
  {
    UpdateState(i);

    if(m_sBatterys[i].m_State == EState::Discharging)
    {
      UpdateCapacity(i);
    }
  }
#ifdef DEBUG
   if(lastScreen >=0)
   {
     Serial.print("State: ");
     Serial.print((int)m_sBatterys[lastScreen].m_State);
     Serial.print(" C: ");
     Serial.print(m_sBatterys[lastScreen].m_Capacity);
     Serial.print(" V: ");
     Serial.print(m_sBatterys[lastScreen].m_Voltage);
     Serial.println();
    }

//    Serial.print("lastScreen: ");
//    Serial.print(lastScreen);
//    Serial.print("     time out: ");
//    Serial.print(screenTimeOut);
//    Serial.print("     time : ");
//    Serial.print(millis());    
     Serial.println();
   	ShiftPWM.PrintInterruptLoad();
#endif

  	UpdateInput();
  if(screenRefresh < millis())
    UpdateDisplay();
    
  if(screenTimeOut > 0 && screenRefresh < millis())
  {
    screenRefresh = millis() + SCREEN_REFRESH;
  }

  
}

//void timerIsr()
//{
//  Timer1.setPwmDuty(13,205);
////  digitalWrite(13, digitalRead(13)^1);
//}
