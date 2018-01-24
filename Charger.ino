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

#define DEBUG

#define SCREEN_TIMEOUT 5000
#define MAIN_SCREEN -1
#define SCREEN_OFF -2

#define BAT_COUNT 20
#define BAT_READ_COUNT 10
const float m_cfBatteryDischargeVoltage = 4.1;
const float m_cfBatteryCutOffVoltage = 2.8;

#define MUX_CH_COUNT 16
#define MUX_S0 42
#define MUX_S1 44
#define MUX_S2 46
#define MUX_S3 48
#define MUX_SIG A1
#define BAT_17 A2
#define BAT_18 A3
#define BAT_19 A4
#define BAT_20 A5

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);



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
unsigned char maxBrightness = 255;
unsigned char pwmFrequency = 200; //DO NOT CHANGE pwmFrequency after calibrating pwmDuty and voltageCalibration
const int numRegisters = 2;

short lastBatteryActivated = -1;
short screenTimeOut = SCREEN_TIMEOUT;



//mesured to draw 1000 mA
const float pwmDuty[8 * numRegisters] = {
    0.2845,         //70,//1
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
    0.9515,//1
    0.9581,//2
    0.9603,//3
    0.9649,//4
    0.9515,//5
    0.9559,//6
    0.9559,//7
    0.9559,//8
    0.9581,//9
    0.9581,//10
    0.9559,//11
    0.9581,//12
    0.9537,//13
    0.9671,//14
    0.9671,//15
    0.9603,//16
    0.9581,//17
    0.9515,//18
    0.9537,//19
    0.9493//20
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
    m_sBatterys[nBat].m_Time = millis() - m_sBatterys[nBat].m_LastTime;
    m_sBatterys[nBat].m_Capacity += (m_sBatterys[nBat].m_Current * (float)m_sBatterys[nBat].m_Time) / 3600000.0; //A*deltaTime/1hour
    m_sBatterys[nBat].m_LastTime = millis();
}

void UpdateState(int nBat)
{
  UpdateBatteryVoltage(nBat);
  
  if(m_sBatterys[nBat].m_State != EState::Empty && m_sBatterys[nBat].m_Voltage < m_cfBatteryCutOffVoltage - 0.2)
  {
    m_sBatterys[nBat].m_State = EState::Empty;

    lastBatteryActivated = -1;
    //Go through the batterys and set next charged, or return to main screen
    for (int i = 0; i < BAT_COUNT; ++i)
    {
        if(m_sBatterys[i].m_State == EState::Finished)
        {
          lastBatteryActivated = i;
          break;
        }
    }

  }else if(m_sBatterys[nBat].m_State == EState::Discharging && m_sBatterys[nBat].m_Voltage <= m_cfBatteryCutOffVoltage)
  {
    m_sBatterys[nBat].m_State = EState::Finished;
    lastBatteryActivated = i;
  }else if(m_sBatterys[nBat].m_State == EState::Empty)
  {
    lastBatteryActivated = nBat; // to display LOW if not charged battery added, and do see its voltages when adding

    if(m_sBatterys[nBat].m_Voltage >= m_cfBatteryDischargeVoltage)
    {
      m_sBatterys[nBat].m_State = EState::Discharging;
      m_sBatterys[nBat].m_LastTime = millis();
      m_sBatterys[nBat].m_Capacity = 0.0;
    }else if(m_sBatterys[nBat].m_Voltage > m_cfBatteryCutOffVoltage)
    {
      lastBatteryActivated = nBat; // to display LOW if not charged battery added, and do see its voltages when adding

      m_sBatterys[nBat].m_State = EState::Low;
    }
  }
}

void UpdateDisplay()
{
  if(screenTimeOut > 0)
  {
    screenTimeOut -= millis();
  }

  if(lastBatteryActivated == MAIN_SCREEN) // MAIN screen
  {
    lastBatteryActivated = SCREEN_OFF; // To prevent clearing screen repeatedly
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Battery Tester");
    lcd.setBacklight(LOW);

  }else if(lastBatteryActivated >= 0) // Display the last active battery for some time then reset to MAIN screen

    if(m_sBatterys[lastBatteryActivated].m_State == EState::Low)
    {
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Battery ");
      lcd.setCursor(11, 0);
      lcd.print(lastBatteryActivated + 1);
      lcd.setCursor(3, 1);
      lcd.print("LOW: ");
      lcd.setCursor(8, 1);
      lcd.print(m_sBatterys[lastBatteryActivated].m_Voltage);
      lcd.setCursor(12, 1);
      lcd.print("V");
    }else if(m_sBatterys[lastBatteryActivated].m_State == EState::Finished)
    {
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Battery ");
      lcd.setCursor(11, 0);
      lcd.print(lastBatteryActivated + 1);
      lcd.setCursor(7, 1);
      lcd.rightToLeft();
      lcd.print(m_sBatterys[lastBatteryActivated].m_Capacity);
      lcd.leftToRight();
      lcd.setCursor(9, 1);
      lcd.print("mAh");
    }

//   lcd.print  (m_sBatterys[19].m_Voltage);
  }else if(screenTimeOut <= 0)
  {
    lastBatteryActivated = SCREEN_OFF;
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

  screenTimeOut = SCREEN_TIMEOUT;

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
  ShiftPWM.Start(pwmFrequency, 255);
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
      ShiftPWM.SetOne(i, (unsigned char)((float)pwmFrequency * pwmDuty[i]));
  }
  delay(1);
  
  for (int i = 0; i < BAT_COUNT; ++i )
  {
    UpdateState(i);

    if(m_sBatterys[i].m_State == EState::Discharging)
    {
      UpdateCapacity(i);
    }

#ifdef DEBUG
   if(i%4 == 0) Serial.println();
   Serial.print("State: ");
   Serial.print((int)m_sBatterys[i].m_State);
   Serial.print(" C: ");
   Serial.print(m_sBatterys[i].m_Capacity);
   Serial.print(" V: ");
   Serial.print(m_sBatterys[i].m_Voltage);
   Serial.print(", ");
#endif
  }
#ifdef DEBUG
  Serial.println();
  Serial.print("*******************************************************************************");
#endif

  UpdateDisplay();
}

//void timerIsr()
//{
//  Timer1.setPwmDuty(13,205);
////  digitalWrite(13, digitalRead(13)^1);
//}
