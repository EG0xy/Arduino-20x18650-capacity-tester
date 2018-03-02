#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <CD74HC4067.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// Connection
//A0 - LCD Keypad shield - 5 keys array
//~0 - Right
//~130 - Up
//~306 - Down
//~479 - Left
//~719 - Select
//
//A1 - Multiplexer HP4067 - 1 (SIG pin) Battery (16) + Voltage Reading
//A2 - Multiplexer HP4067 - 2 (SIG pin) Battery (16) + Voltage Reading
//A3 - Multiplexer HP4067 - 3 (SIG pin) Battery (16) + Voltage Reading
//A4 - Multiplexer HC4067 - 4 (SIG pin) Battery (16) + Voltage Reading
//
//D43 - Multiplexer HP4067 - 1 (EN pin) Battery (16) + Voltage Reading
//D45 - Multiplexer HP4067 - 2 (EN pin) Battery (16) + Voltage Reading
//D47 - Multiplexer HP4067 - 3 (EN pin) Battery (16) + Voltage Reading
//D49 - Multiplexer HC4067 - 4 (EN pin) Battery (16) + Voltage Reading
//
//D42 - Multiplexer HC4067 (S0 pin) Read control
//D44 - Multiplexer HC4067 (S1 pin) Read control  <<--------------- EN pin should be set on multiplexer in use to LOW before seting read channel
//D46 - Multiplexer HC4067 (S2 pin) Read control
//D48 - Multiplexer HC4067 (S3 pin) Read control
//
//2 x PCA9685PW Adresses in l2C: 0x40 and 0x41 
//16x2 display Adress in l2C: 0x3F
//
//0-15	MUX 1 
//0-3		MUX 2 - Charged read out from Led R1 in TP4056 (NC)
//4		MUX 2 - NC (not connected)
//5-15	MUX 2
//0-8		MUX 3 - Voltage read out from batteries
//9-15	MUX 3
//0-12	MUX 4 - Voltage read out from ressistor
//
//81.85931902590525 - 0.11406781021018966x + 0.00007701200364055409x2 - 2.666253770521e-8x3 + 3.54777456e-12x4 - formula for % diff from OPUS

#define USE_SERIAL
#define DISABLE_CHARGING


#define MUX_COUNT 4
#define MUX_CH_COUNT 16

#define MUX_S0 42
#define MUX_S1 44
#define MUX_S2 46
#define MUX_S3 48
#define MUX_1_SIG A1
#define MUX_2_SIG A2
#define MUX_3_SIG A3
#define MUX_4_SIG A4
#define MUX_1_EN 43
#define MUX_2_EN 45
#define MUX_3_EN 47
#define MUX_4_EN 49

#define PWM_DRIVER_COUNT 2
#define PWM_CH_COUNT 16
#define MAX_PWM 4095

#define PWM_OFF 0
#define PWM_CHARGE 0
#define PWM_ON	750 //~1000mA

//Button input
#define INPUT_PIN A0

#define RESISTANCE_TIMER 2000 //waits for millliseconds until another mesurement for resistance calculation

#define BAT_COUNT 20
#define BAT_READ_COUNT 10
const float m_cfBatteryDischargeVoltage = 4.f;//V
const float m_cfBatteryCutOffVoltage = 2.85f;//V
const float m_cfRechargeVoltage = 3.75f;//V
const float m_cfPreDischargeVoltage = 3.8f;//V
const float m_cfDischargeCurrent = 1000.f;//mA
static int currentMUX = 0;

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address for a 16 chars and 2 line display
//Multiplexer control
CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);

Adafruit_PWMServoDriver pwmDriver[PWM_DRIVER_COUNT] = {Adafruit_PWMServoDriver(0x40),Adafruit_PWMServoDriver(0x41)};

enum class EBatteryState :int
{
	Empty = 0,//Voltage visible if lower then m_cfBatteryDischargeVoltage
	Low,
	PreDischarge, //for TP4056 to charge up the batteries before disharging batteries need to be lower then 4V for TP4056 to start carging (Adding load until lower then 4V)
	Charging, //charging from TP4056 
	Resistance,
	Discharging,
	Recharging, //to 3.7V
	Finished//results
};

uint8_t muxSig[MUX_COUNT] = { MUX_1_SIG, MUX_2_SIG, MUX_3_SIG, MUX_4_SIG };
int muxEN[MUX_COUNT] = { MUX_1_EN, MUX_2_EN, MUX_3_EN, MUX_4_EN };

struct Data
{
	int chanel;
	int id;

	Data() :chanel(0), id(0){};
};

struct ResistanceData {
	float initVoltage;
	unsigned long timer;

	ResistanceData() :initVoltage(0.f), timer(0) {};
};

struct Batterie
{
	Data batterieReadPinsData;
	Data ressistorReadPinsData;
	Data pwmPinsData;
	ResistanceData resistanceData;
	EBatteryState state;
	unsigned int pwm;
	float capacity;
	float resistance;
	float voltage;
	float loadVoltage;
	float dischargeCurrent; //this is for if you need to set seperate discharge rates on slots
	unsigned long lasTime;

	Batterie() :state(EBatteryState::Empty), pwm(0), capacity(0.f), resistance(0.f), voltage(0.f), loadVoltage(0.f), dischargeCurrent(m_cfDischargeCurrent), lasTime(0) {};
	void Reset()
	{
		resistanceData.initVoltage = 0.f;
		resistanceData.timer = 0;
		state = EBatteryState::Empty;
		pwm = PWM_OFF;
		capacity = 0.f;
		resistance = 0.f;
		lasTime = 0;

		pwmDriver[pwmPinsData.id].setPWM(pwmPinsData.chanel, 0, pwm);
	};
};

Batterie batteries[BAT_COUNT];

//read voltage from analog pins
float getVoltage(const Data& pinData, float voltageMultiplierValue = 1.0) {
	float sample = 0.0;
	//Enabling the proper multiplexer for read sequences
	if (currentMUX != pinData.id)
	{
		digitalWrite(muxEN[currentMUX], HIGH);
		digitalWrite(muxEN[pinData.id], LOW);
		currentMUX = pinData.id;
	}
	mux.channel(pinData.chanel);
	delay(1);
	//analogRead(muxSig[pinData.id]);//to reduce errors for bad reads skip first read
	for (uint8_t i = 0; i < BAT_READ_COUNT; i++) {
		sample += analogRead(muxSig[pinData.id]);
	}
	sample = sample / (float)BAT_READ_COUNT;

	//voltage is read from voltage divider so multiplication by 2 is necesary
	return (float)sample * voltageMultiplierValue * ((readVcc() * 0.001f) / 1023.0);
}
void UpdateBatteries()
{
	//load voltage and batterie voltage reads in separate loops in order to minimize the multiplexer switching
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].voltage = getVoltage(batteries[i].batterieReadPinsData, 2.f * 0.97f);
	}
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].loadVoltage = getVoltage(batteries[i].ressistorReadPinsData);// , 1.006f);
	}
}
void UpdateCapacity(float current,int batID)
{
	unsigned long nDeltaTime = millis() - batteries[batID].lasTime;
	batteries[batID].capacity += current * (float)nDeltaTime / 3600000.f;
	batteries[batID].lasTime += nDeltaTime;
}

void SetPWM(float current, int batID) {
  if(current > 0 && current != batteries[batID].dischargeCurrent)
  {
    //for faster current equality
    int currentDiff = batteries[batID].dischargeCurrent - current;
	int pwmChange = map((int)(abs(currentDiff) *  0.2f), 0, 2000, 0, MAX_PWM);

    if(currentDiff > 0 && batteries[batID].pwm < MAX_PWM)
    {
      batteries[batID].pwm += max(pwmChange, 1);
    }else if(currentDiff < 0 && batteries[batID].pwm > 0)
    {
      batteries[batID].pwm -= max(pwmChange, 1);
    }

	if (batteries[batID].pwm > MAX_PWM)
		batteries[batID].pwm = MAX_PWM;
	if (batteries[batID].pwm < PWM_OFF)
		batteries[batID].pwm = PWM_OFF;

     pwmDriver[batteries[batID].pwmPinsData.id].setPWM(batteries[batID].pwmPinsData.chanel, 0, batteries[batID].pwm);
  }
}

long readVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = _BV(MUX3) | _BV(MUX2);
#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif 
#if defined(__AVR_ATmega2560__)
	ADCSRB &= ~_BV(MUX5); // Without this the function always returns -1 on the ATmega2560
#endif

	ADCSRA |= _BV(ADEN);  // Enable the ADC

	delay(2); // Wait for Vref to settle

	ADCSRA |= _BV(ADSC); // Start conversion

	while (bit_is_set(ADCSRA, ADSC)); // Detect end-of-conversion

	uint8_t low = ADCL; // must read ADCL first - it then locks ADCH 
	uint8_t high = ADCH; // unlocks both

	long result = (high << 8) | low;

	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

	return result; // Vcc in millivolts
}

void setup() {
	Wire.begin();
	Serial.begin(9600);
	lcd.begin(16, 2);
	lcd.setBacklight(LOW);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.printstr("Hello");

	//*********************** Multiplexer HC4067 setup
	pinMode(MUX_1_SIG, INPUT);
	pinMode(MUX_2_SIG, INPUT);
	pinMode(MUX_3_SIG, INPUT);
	pinMode(MUX_4_SIG, INPUT);

	pinMode(MUX_1_EN, OUTPUT);
	pinMode(MUX_2_EN, OUTPUT);
	pinMode(MUX_3_EN, OUTPUT);
	pinMode(MUX_4_EN, OUTPUT);

	pinMode(MUX_S0, OUTPUT);
	pinMode(MUX_S1, OUTPUT);
	pinMode(MUX_S2, OUTPUT);
	pinMode(MUX_S3, OUTPUT);

	digitalWrite(MUX_S0, LOW);
	digitalWrite(MUX_S1, LOW);
	digitalWrite(MUX_S2, LOW);
	digitalWrite(MUX_S3, LOW);

	digitalWrite(MUX_1_EN, LOW);
	digitalWrite(MUX_2_EN, LOW);
	digitalWrite(MUX_3_EN, LOW);
	digitalWrite(MUX_4_EN, LOW);

	//buttons
	pinMode(A0, INPUT);

	//PWM
	for (size_t i = 0; i < PWM_DRIVER_COUNT; i++)
	{
		pwmDriver[i].begin();
		pwmDriver[i].setPWMFreq(1600);
	}

	//  0 - 15	MUX 0
	//	0 - 3	MUX 1 - Charged read out from Led R1 in TP4056(NC)
	//	4		MUX 1 - NC(not connected)
	//	5 - 15	MUX 1
	//	0 - 8	MUX 2 - Voltage read out from batteries
	//	9 - 15	MUX 2
	//	0 - 12	MUX 3 - Voltage read out from ressistor

	int muxBVNum = 1;
	int muxCH_BVNum = 5;
	int muxRVNum = 2;
	int muxCH_RVNum = 9;
	int pwmPICNum = 0;
	int pwmCHNum = 0;
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i] = Batterie();

		if (muxCH_BVNum >= MUX_CH_COUNT)
		{
			muxCH_BVNum = 0;
			++muxBVNum;
		}

		batteries[i].batterieReadPinsData.chanel = muxCH_BVNum++;
		batteries[i].batterieReadPinsData.id = muxBVNum;

		if (muxCH_RVNum >= MUX_CH_COUNT)
		{
			muxCH_RVNum = 0;
			++muxRVNum;
		}

		batteries[i].ressistorReadPinsData.chanel = muxCH_RVNum++;
		batteries[i].ressistorReadPinsData.id = muxRVNum;

		if (pwmCHNum >= PWM_CH_COUNT)
		{
			pwmCHNum = 0;
			++pwmPICNum;
		}

		batteries[i].pwmPinsData.chanel = pwmCHNum++;
		batteries[i].pwmPinsData.id = pwmPICNum;

		batteries[i].pwm = PWM_OFF;
		pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
	}
}

void loop() {
	UpdateBatteries();
	UpdateState();
}

void UpdateState()
{
	for (size_t i = 0; i < BAT_COUNT; ++i)
	{

		if (batteries[i].state != EBatteryState::Empty && batteries[i].voltage <= m_cfBatteryCutOffVoltage - 0.5f)
		{
			batteries[i].Reset();
#ifdef USE_SERIAL
			Serial.print("Batterie : ");
			Serial.print(i + 1);
			Serial.print(" RESET");
			Serial.println();
			break;
#endif
		}

		switch (batteries[i].state)
		{
			case EBatteryState::Empty:
			{
				if (batteries[i].voltage >= m_cfBatteryDischargeVoltage)
				{
					batteries[i].state = EBatteryState::PreDischarge;
				}
				else if (batteries[i].voltage <= m_cfBatteryCutOffVoltage)
				{
					break;
				}
				else {
					batteries[i].state = EBatteryState::Low;
				}
			}
			case EBatteryState::Low:
			{
				if (batteries[i].state == EBatteryState::Low)
				{
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
			case EBatteryState::PreDischarge:
			{
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" PREDISCHARGING");
				Serial.println();
#endif
#ifndef DISABLE_CHARGING

				if (batteries[i].pwm == PWM_OFF)
				{
					batteries[i].pwm = PWM_ON;
					pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
					break;
				}
				else if (batteries[i].voltage < m_cfPreDischargeVoltage)
				{
					batteries[i].pwm = PWM_OFF;
					batteries[i].state = EBatteryState::Charging;
				}
				else
				{
					break;
				}
#else
				batteries[i].state = EBatteryState::Charging;
#endif
				
			}
			case EBatteryState::Charging:
			{
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" CHARGING");
				Serial.println();
#endif
#ifndef DISABLE_CHARGING

				if (batteries[i].pwm == PWM_OFF)
				{
					batteries[i].pwm = PWM_CHARGE;
					pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
				}
				else if (true)//TODO: ADD triger for charged completed (STDBY pin on TP4056)
				{
					batteries[i].pwm = PWM_OFF;
					pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
					batteries[i].state = EBatteryState::Resistance;
				}
				
				break;
				
#else
				batteries[i].state = EBatteryState::Resistance;
#endif
			}
			case EBatteryState::Resistance:
			{
				if (batteries[i].resistanceData.timer == 0)
				{

					batteries[i].resistanceData.initVoltage = batteries[i].voltage;
					batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;
#ifdef USE_SERIAL
					Serial.print("Batterie : ");
					Serial.print(i + 1);
					Serial.print(" INIT voltage: ");
					Serial.print(batteries[i].voltage);
					Serial.print(" INIT resistance timer");
					Serial.println();
#endif
				}else if (batteries[i].resistance == 0.0 && batteries[i].resistanceData.timer <= millis())
				{
					float vDiff = batteries[i].resistanceData.initVoltage - batteries[i].voltage;
					batteries[i].resistance = vDiff * 1000.f / batteries[i].loadVoltage; // result in miliohms
#ifdef USE_SERIAL
					Serial.print("Batterie : ");
					Serial.print(i+1);
					Serial.print(" resistance : ");
					Serial.print(batteries[i].resistance);
					Serial.print(" mOhm");
					Serial.print(" init V : ");
					Serial.print(batteries[i].resistanceData.initVoltage);
					Serial.print(" load V : ");
					Serial.print(batteries[i].voltage);
					Serial.print(" current A : ");
					Serial.print(batteries[i].loadVoltage);
					Serial.println();
#endif
				}

				batteries[i].state = EBatteryState::Discharging;
			}
			case EBatteryState::Discharging:
			{
				if (batteries[i].voltage <= m_cfBatteryCutOffVoltage)
				{
#ifdef USE_SERIAL
					Serial.print("Batterie : ");
					Serial.print(i + 1);
					Serial.print(" DISCHARGE completed");
					Serial.println();
#endif
					batteries[i].pwm = PWM_OFF;
					pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
					batteries[i].state = EBatteryState::Recharging;
#ifdef DISABLE_CHARGING
					//remove when chargers working
					batteries[i].voltage = m_cfRechargeVoltage;
#endif
				}
				else
				{
					if (batteries[i].pwm == PWM_CHARGE || batteries[i].pwm == PWM_OFF)
					{
						batteries[i].pwm = PWM_ON;
						batteries[i].lasTime = millis();
						pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
						batteries[i].state = EBatteryState::Discharging;
						break;
					}

					if (batteries[i].resistance == 0.0 && batteries[i].resistanceData.timer <= millis())
					{
						batteries[i].state = EBatteryState::Resistance;
					}

					UpdateCapacity(batteries[i].loadVoltage * 1000.f, i);
					SetPWM(batteries[i].loadVoltage * 1000.f, i);
#ifdef USE_SERIAL
					Serial.print("Batterie : ");
					Serial.print(i + 1);
					Serial.print(" Capacity: ");
					Serial.print(batteries[i].capacity);
					Serial.println();
#endif
				}

				break;
			}
			case EBatteryState::Recharging:
			{
#ifndef DISABLE_CHARGING
				if (batteries[i].voltage >= m_cfRechargeVoltage)
				{
					batteries[i].pwm = PWM_OFF;
					pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
					batteries[i].state = EBatteryState::Finished;
				}
				else
				{

					if (batteries[i].pwm == PWM_OFF)
					{
						batteries[i].pwm = PWM_CHARGE;
						pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
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
				batteries[i].state = EBatteryState::Finished;
#endif
			} 
			case EBatteryState::Finished:
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
	}
}
