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
        ~130 - Down
        ~306 - Up
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
  float m_Current;
  unsigned long m_Time;
  unsigned long m_LastTime;


  Battery(): m_State(EState::Empty), m_Capacity(0.0), m_Voltage(0.0), m_Current(1000.0), m_Time(0), m_LastTime(millis()){};  
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



//mesured to draw 1000 mA
const float pwmDuty[8 * numRegisters] = {
    0.2905,         //70,//1
    0.2649,         //65,//2
    0.2365,         //59,//3
    0.2620,         //65,//4
    0.2195,         //55,//5
    0.2500,         //63,//6
    0.2510,         //64,//7
    0.2605,         //64,//8
    0.2323,         //59,//9
    0.2176,         //56,//10
    0.2126,         //55,//11
    0.2235,         //57,//12
    0.2078,         //53,//13
    0.2048,         //53,//14
    0.2196,         //56,//15
    0.2117,         //54,//16
    0.2352,         //60,//17
    0.2078,         //53,//18
    0.2431,         //62,//19
    0.2392,         //61,//20
    0.0,            //0,//21 -- not used
    0.0,            //0,//22 -- not used
    0.0,            //0,//23 -- not used
    0.0             //0 //24 -- not used
};

//readings calibrated by my multimeter
float voltageCalibration[20]=
{
    0.9523,//1
    0.9539,//2
    0.9571,//3
    0.9566,//4
    0.9462,//5
    0.9504,//6
    0.9504,//7
    0.9507,//8
    0.9534,//9
    0.9533,//10
    0.9510,//11
    0.9534,//12
    0.9494,//13
    0.9607,//14
    0.9619,//15
    0.9540,//16
    0.9510,//17
    0.9481,//18
    0.9486,//19
    0.9443//20
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
    m_sBatterys[nBat].m_Capacity += (m_sBatterys[nBat].m_Current * (float)nDeltaTime) / 3600000.0; //A*deltaTime/1hour
    m_sBatterys[nBat].m_LastTime = millis();

    m_sBatterys[nBat].m_Time += nDeltaTime;
}

void UpdateState(int nBat)
{
  UpdateBatteryVoltage(nBat);
  
  if(m_sBatterys[nBat].m_State != EState::Empty && m_sBatterys[nBat].m_Voltage < m_cfBatteryCutOffVoltage - 0.2)
  {
    m_sBatterys[nBat].m_State = EState::Empty;

    lastScreen = MAIN_SCREEN;
    EnableLCD();
  }else if(m_sBatterys[nBat].m_State == EState::Discharging && m_sBatterys[nBat].m_Voltage <= m_cfBatteryCutOffVoltage)
  {
#ifdef DEBUG
    Serial.print("Finished SET");
    Serial.println();
#endif
    m_sBatterys[nBat].m_State = EState::Finished;
    lastScreen = nBat;
    EnableLCD();

  }else if (lastScreen == MAIN_SCREEN && (m_sBatterys[nBat].m_State == EState::Finished || m_sBatterys[nBat].m_State == EState::Low)) //When battery is pulled out from some slot, next Finished or Low will be searched for.
  {
    lastScreen = nBat;
    EnableLCD();    
  }else if(m_sBatterys[nBat].m_State == EState::Empty)
  {
    if(m_sBatterys[nBat].m_Voltage >= m_cfBatteryDischargeVoltage )//&& lastScreen <= MAIN_SCREEN)
    {
#ifdef DEBUG
    Serial.print("Discharge SET");
    Serial.println();
#endif
      lastScreen = nBat;
      m_sBatterys[nBat].m_State = EState::Discharging;
      m_sBatterys[nBat].m_LastTime = millis();
      m_sBatterys[nBat].m_Capacity = 0.0;
      EnableLCD();
    }else if(m_sBatterys[nBat].m_Voltage > m_cfBatteryCutOffVoltage)
    {
#ifdef DEBUG
    Serial.print("Low SET");
    Serial.println();
#endif
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
  }else if(lastScreen >= 0)// && screenTimeOut > millis()) // Display the last active battery for some time then reset to MAIN screen
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
      lcd.print((int)m_sBatterys[lastScreen].m_Capacity);
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
      lcd.print((int)m_sBatterys[lastScreen].m_Capacity);
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

  ShiftPWM.SetAll(0);
  for (int i = 0; i < min(BAT_COUNT,8 * numRegisters); ++i )
  {
    if(m_sBatterys[i].m_State == EState::Discharging)
      ShiftPWM.SetOne(i, (unsigned char)((float)maxBrightness * pwmDuty[i]));
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
   
#endif

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
