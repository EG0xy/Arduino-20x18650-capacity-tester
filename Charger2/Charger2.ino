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

//#define USE_SERIAL
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
#define PWM_ON 1547 //~1000mA
#define PWM_I_1 399 // ~250.38mA
#define PWM_I_2 1547 // ~1000.49mA

//Button input
#define INPUT_PIN A0

#define RESISTANCE_TIMER 1500 //waits for millliseconds until another mesurement for resistance calculation

#define RESISTANCE_MULTIPLIER 0.01056421f //1000mA mesured

#define BAT_COUNT 20
#define BAT_READ_COUNT 10
#ifdef DISABLE_CHARGING
const float m_cfBatteryDischargeVoltage = 4.1f;//V
#else
const float m_cfBatteryDischargeVoltage = 4.2f;//V
#endif // DISABLE_CHARGING
const float m_cfBatteryCutOffVoltage = 2.8f;//V
const float m_cfRechargeVoltage = 3.75f;//V
const float m_cfPreDischargeVoltage = 3.8f;//V
const float m_cfDischargeCurrent = 1000.f;//mA
static int currentMUX = 0;

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

#define LCD_ON -1
#define LCD_OFF -2

#define LCD_TIMEOUT 10000
#define LCD_REFRESH 500

short lastScreen = LCD_ON;
unsigned long lcdTimeOut = LCD_TIMEOUT;
unsigned long screenRefresh = LCD_REFRESH;
unsigned long batteryLastInfoTimer = 0;
bool bLastInfoTrigered = false;

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address for a 16 chars and 2 line display
									//Multiplexer control
CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);

Adafruit_PWMServoDriver pwmDriver[PWM_DRIVER_COUNT] = { Adafruit_PWMServoDriver(0x40),Adafruit_PWMServoDriver(0x41) };

enum class EBatteryState :int
{
	Empty = 0,
	Low,//batteries always skips this state if DISABLE_CHARGING undefined (precharging enabled)
	PreDischarge, //for TP4056 to charge up the batteries before disharging batteries need to be lower then 4V for TP4056 to start carging (Adding load until lower then 4V)
	Charging, //charging from TP4056 
	Resistance,
	Discharging,
	Recharging, //to 3.7V
	Finished//results
};

uint8_t muxSig[MUX_COUNT] = { MUX_1_SIG, MUX_2_SIG, MUX_3_SIG, MUX_4_SIG };
int muxEN[MUX_COUNT] = { MUX_1_EN, MUX_2_EN, MUX_3_EN, MUX_4_EN };

struct MuxData{
	int chanel;
	int id;

	MuxData() :chanel(0), id(0) {};
};

struct ResistanceData {
	float voltages[2] = { 0.f,0.f };
	float currents[2] = { 0.f,0.f };
	unsigned long timer;
	byte currentMesurement;

	ResistanceData() :timer(0), currentMesurement(0) {};
};

struct Battery
{
	MuxData temperatureReadPinsData;
	MuxData batterieReadPinsData;
	MuxData ressistorReadPinsData;
	MuxData pwmPinsData;
	ResistanceData resistanceData;
	EBatteryState state;
	unsigned int pwm;
	float temperature;
	float capacity;
	float resistance;
	float voltage;
	float loadVoltage;
	float dischargeCurrent; //this is for if you need to set seperate discharge rates on slots
	unsigned long lasTime;

	Battery() :state(EBatteryState::Empty), pwm(PWM_OFF), temperature(0.f), capacity(0.f), resistance(0.f), voltage(0.f), loadVoltage(0.f), dischargeCurrent(m_cfDischargeCurrent), lasTime(0) {};
	void Reset()
	{
		resistanceData.voltages[0] = 0.f;
		resistanceData.voltages[1] = 0.f;
		resistanceData.currents[0] = 0.f;
		resistanceData.currents[1] = 0.f;
		resistanceData.currentMesurement = 0;
		resistanceData.timer = 0;
		state = EBatteryState::Empty;
		pwm = PWM_OFF;
		capacity = 0.f;
		resistance = 0.f;
		lasTime = 0;

		pwmDriver[pwmPinsData.id].setPWM(pwmPinsData.chanel, 0, pwm);
	};
};

Battery batteries[BAT_COUNT];

float voltageReadCalibration[BAT_COUNT] =
{	
	0.01991374f,
	0.01728894f,
	0.0145641444f,
	0.0091367661f,
	0.0262232668f,
	0.0202853987f,
	0.0205658379f,
	0.0202965867f,
	0.017290452f,
	0.0178396008f,
	0.0205772194f,
	0.0153635926f,
	0.020852474f,
	0.0085018169f,
	0.0070967872f,
	0.016547677f,
	0.0179664988f,
	0.0233406144f,
	0.0233406144f,
	0.0255810778f
};

void EnableLCD(float timeMultiplier = 1.f)
{
	lcd.setBacklight(LOW);
	lcdTimeOut = millis() + (unsigned long)(round((float)LCD_TIMEOUT*timeMultiplier));
}

//read voltage from analog pins
float getVoltage(const MuxData& pinData, float voltageMultiplierValue = 1.0) {
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
	return (float)sample * voltageMultiplierValue * ((readVcc() * 0.001f) / 1024.0);
}

float getCellTemp(const MuxData& pinData, int t = 1)
{	
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

	float R1 = 10000;
	float logR2, R2, T, Tc, Tf;
	float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

	R2 = R1 * (1023.0 / (float)analogRead(muxSig[pinData.id]) - 1.0);
	logR2 = log(R2);
	T = (1.0 / (c1 + c2 * logR2 + c3 * logR2*logR2*logR2));
	return (float)(T - 278.15);
}
unsigned long lastUpdate = 0;
void UpdateBatteries()
{
	if ((millis() - lastUpdate) >= 1000)
	{
		//load voltage and batterie voltage reads in separate loops in order to minimize the multiplexer switching
		for (int i = 0; i < BAT_COUNT; ++i)
		{
			batteries[i].temperature = getCellTemp(batteries[i].temperatureReadPinsData);
#ifdef USE_SERIAL
			Serial.print(batteries[i].temperature);
			Serial.print(" ");
#endif
		}
		lastUpdate = millis();
	}
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].voltage = getVoltage(batteries[i].batterieReadPinsData, 2.f);
		batteries[i].voltage -= batteries[i].voltage * voltageReadCalibration[i];
	}
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].loadVoltage = getVoltage(batteries[i].ressistorReadPinsData);
		batteries[i].loadVoltage += batteries[i].loadVoltage * RESISTANCE_MULTIPLIER;
	}
#ifdef USE_SERIAL
	Serial.println();
#endif
}
void UpdateCapacity(float current, int batID)
{
	unsigned long nDeltaTime = millis() - batteries[batID].lasTime;
	batteries[batID].capacity += current * (float)nDeltaTime / 3600000.f;
	batteries[batID].lasTime += nDeltaTime;
}

void SetPWM(float current, int batID) {
	if (current > 0 && current != batteries[batID].dischargeCurrent)
	{
		int newPwm = batteries[batID].pwm;
		//for faster current equality
		int currentDiff = batteries[batID].dischargeCurrent - current;
		int pwmChange = map((int)(abs(currentDiff) *  0.2f), 0, 2000, 0, MAX_PWM);

		if (currentDiff > 0 && newPwm < MAX_PWM)
		{
			newPwm += max(pwmChange, 1);
		}
		else if (currentDiff < 0 && newPwm > 0)
		{
			newPwm -= max(pwmChange, 1);
		}

		if (newPwm > MAX_PWM)
			newPwm = MAX_PWM;
		if (newPwm < PWM_OFF)
			newPwm = PWM_OFF;

		if (batteries[batID].pwm != newPwm)
		{
			batteries[batID].pwm = newPwm;
			pwmDriver[batteries[batID].pwmPinsData.id].setPWM(batteries[batID].pwmPinsData.chanel, 0, batteries[batID].pwm);
		}
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

byte omega[8] = {
	B00000,
	B01110,
	B10001,
	B10001,
	B10001,
	B01010,
	B11011,
	B00000
};
byte celcius[8] = {
	B00000,
	B10110,
	B01000,
	B01000,
	B01000,
	B01000,
	B00110,
	B00000
};

void setup() {
	Wire.begin();
	Serial.begin(9600);
	lcd.createChar(0, omega);
	lcd.createChar(1, celcius);
	lcd.begin(16, 2);
	UpdateDisplay();

	lastUpdate = millis();

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

	int muxTEMPNum = 0;
	int muxCH_TEMPNum = 0;
	int muxBVNum = 1;
	int muxCH_BVNum = 5;
	int muxRVNum = 2;
	int muxCH_RVNum = 9;
	int pwmPICNum = 0;
	int pwmCHNum = 0;
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i] = Battery();

		if (muxCH_TEMPNum >= MUX_CH_COUNT)
		{
			muxCH_TEMPNum = 0;
			++muxTEMPNum;
		}

		batteries[i].temperatureReadPinsData.chanel = muxCH_TEMPNum++;
		batteries[i].temperatureReadPinsData.id = muxTEMPNum;

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
	UpdateInput();
	UpdateDisplay();
}

void ResetLastInfo()
{
	if (bLastInfoTrigered)
	{
		bLastInfoTrigered = false;
		batteries[lastScreen].state = EBatteryState::Empty;
		batteryLastInfoTimer = millis();
	}
}

void UpdateState()
{
	for (size_t i = 0; i < BAT_COUNT; ++i)
	{
		bool reinserted = bLastInfoTrigered && lastScreen == i && batteryLastInfoTimer > millis() && batteries[lastScreen].voltage > m_cfBatteryCutOffVoltage;
		if (reinserted)
		{
			bLastInfoTrigered = false;
			batteryLastInfoTimer = millis() + LCD_TIMEOUT/2;
		}

		if (!reinserted && batteries[i].state != EBatteryState::Empty && batteries[i].voltage <= m_cfBatteryCutOffVoltage - 0.5f)
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
							batteryLastInfoTimer = millis() + LCD_TIMEOUT/2;
							EnableLCD(0.5f);
						}
					}
					continue;
				}
				else
				{
					if (lastScreen >= 0)
						batteries[lastScreen].Reset();
					batteryLastInfoTimer = millis();
					lastScreen = i;
					EnableLCD();
				}
			}
		}
//		bool reinsertedAfterRemove = bLastInfoTrigered && lastScreen == i &&  batteries[i].voltage > m_cfBatteryCutOffVoltage;
//
//		if (reinsertedAfterRemove)
//		{
//			bLastInfoTrigered = false;
//			batteryLastInfoTimer = millis();
//			batteries[i].Reset();
//		}
////!reinsertedAfterRemove &&
//		if (batteries[i].state != EBatteryState::Empty && batteries[i].voltage <= m_cfBatteryCutOffVoltage - 0.5f)
//		{
//			if (!bLastInfoTrigered || lastScreen != i)
//			{
//				if (lastScreen != i)
//				{
//					batteries[lastScreen].Reset();
//				}
//
//				EnableLCD();//should be before batteryLastInfoTimer for LCD to turn off before batteryLastInfoTimer timer rans out
//				bLastInfoTrigered = true;
//				batteryLastInfoTimer = millis() + LCD_TIMEOUT;
//				lastScreen = i;
//			}
//			else if(lastScreen == i && batteryLastInfoTimer <= millis()) //time ran out for reinsertion of removed battery
//			{
//				bLastInfoTrigered = false;
//				batteries[i].Reset();
//			}
//#ifdef USE_SERIAL
//			Serial.print("Batterie : ");
//			Serial.print(i + 1);
//			Serial.print(" RESET");
//			Serial.println();
//#endif
//		}

		switch (batteries[i].state)
		{
		case EBatteryState::Empty:
		{
			if (batteries[i].voltage >= m_cfBatteryDischargeVoltage)
			{
				batteries[i].state = EBatteryState::Resistance;
				break;
			}
#ifndef DISABLE_CHARGING
			else if (batteries[i].voltage >= m_cfPreDischargeVoltage && batteries[i].voltage < m_cfBatteryDischargeVoltage)
			{
				batteries[i].state = EBatteryState::PreDischarge;
			}
#endif
			else if(batteries[i].voltage >= m_cfBatteryCutOffVoltage){
				batteries[i].state = EBatteryState::Low;

				if (!bLastInfoTrigered)
				{
					EnableLCD();
					lastScreen = i;
				}
			}
			else
			{
				break;
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
				batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;
				
				if (!bLastInfoTrigered)
				{
					EnableLCD();
					lastScreen = i;
				}

				pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, PWM_I_1);
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" INIT resistance timer");
				Serial.print(" --- VCC : ");
				Serial.print(readVcc() * 0.001f);
				Serial.println();
#endif
			}
			else if (batteries[i].resistance == 0.0 && batteries[i].resistanceData.timer <= millis())
			{
				batteries[i].resistanceData.voltages[batteries[i].resistanceData.currentMesurement] = batteries[i].voltage;
				batteries[i].resistanceData.currents[batteries[i].resistanceData.currentMesurement] = batteries[i].loadVoltage;

				++batteries[i].resistanceData.currentMesurement;

				if (batteries[i].resistanceData.currentMesurement >= 2)
				{
					//IEC 61960-2003
					batteries[i].resistance = 1000.f * fabsf(batteries[i].resistanceData.voltages[1] - batteries[i].resistanceData.voltages[0]) / fabsf(batteries[i].resistanceData.currents[1] - batteries[i].resistanceData.currents[0]);

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
					pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, PWM_I_2);
				}
			}

			batteries[i].state = EBatteryState::Discharging;
		}
		case EBatteryState::Discharging:
		{
			if (batteries[i].voltage <= m_cfBatteryCutOffVoltage && (lastScreen == i?!bLastInfoTrigered:true))
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
				batteries[i].voltage = m_cfRechargeVoltage;
#endif
			}
			else
			{
				if (batteries[i].pwm == PWM_CHARGE || batteries[i].pwm == PWM_OFF)
				{
					batteries[i].pwm = PWM_ON;
					batteries[i].lasTime = millis();
					if (batteries[i].resistanceData.currentMesurement >= 2)//Do not change PWM while resistance test running
						pwmDriver[batteries[i].pwmPinsData.id].setPWM(batteries[i].pwmPinsData.chanel, 0, batteries[i].pwm);
					batteries[i].state = EBatteryState::Discharging;
					break;
				}

				if (batteries[i].resistanceData.currentMesurement < 2 && batteries[i].resistanceData.timer <= millis())
				{
					batteries[i].state = EBatteryState::Resistance;
				}

				UpdateCapacity(batteries[i].loadVoltage * 1000.f, i);
				if(batteries[i].resistanceData.currentMesurement >=2)//Do not change PWM while resistance test running
					SetPWM(batteries[i].loadVoltage * 1000.f, i);
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
			if (!bLastInfoTrigered && lastScreen <= LCD_ON)//Finished screen is shown
			{
				EnableLCD();
				lastScreen = i;
			}
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

int PrioritySelect(int step, int nLast)
{
	int result = nLast;
	if (result < 0)
	{
		result = 0;
	}
	else
	{
		ResetLastInfo();
	}

	int nCount = 0;
	bool bFound = false;
	EBatteryState searchState = EBatteryState::Finished;

#ifdef DEBUG
	Serial.print("Entering Priority Selecttion loop");
	Serial.println();
#endif
	do {
		nCount += 1;

		result = result + step;

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

		if (!bFound && nCount == BAT_COUNT && searchState == EBatteryState::Finished)
		{
#ifdef DEBUG
			Serial.print("Priority Selecttion loop: switch to Discharging");
			Serial.println();
#endif
			searchState = EBatteryState::Discharging;
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
}

void UpdateInput()
{
	//Disable controlls for when battery is removed
	if (bLastInfoTrigered) return;

	int readkey = analogRead(0);

#ifdef DEBUG
	Serial.print("Read Value: ");
	Serial.print(readkey);
	Serial.println();
#endif

	if (activeButton == EKeyID::None)
	{
		// if(readkey<30) {
		// 	activeButton = EKeyID::Right;
		// }
		if (readkey >= 30 && readkey<160) {
			activeButton = EKeyID::Up;
		}
		if (readkey >= 160 && readkey<330) {
			activeButton = EKeyID::Down;
		}
		// if(readkey>= 330 && readkey<500) {
		// 	activeButton = EKeyID::Left;
		// }
		// if(readkey>= 500 && readkey<740) {
		// 	activeButton = EKeyID::Select;
		// }

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
		
		if (lastScreen <= LCD_ON)
		{
			InputActive = true;
			lastScreen = PrioritySelect(1, lastScreen);
			return;
		}
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
				EnableLCD();//refreshing timer
				lastScreen = PrioritySelect(1, lastScreen);
				break;
			}
			case EKeyID::Down:
			{
				InputActive = true;
#ifdef DEBUG
				Serial.print("DOWN");
				Serial.println();
#endif
				EnableLCD();//refreshing timer
				lastScreen = PrioritySelect(-1, lastScreen);
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
			case EBatteryState::Empty:
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
			case EBatteryState::Low:
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
			case EBatteryState::PreDischarge:
			{
				lcd.clear();
				lcd.setCursor(3, 0);
				lcd.print("Battery ");
				lcd.setCursor(11, 0);
				lcd.print(lastScreen + 1);
				lcd.setCursor(1, 1);
				lcd.print("PreDis");
				lcd.setCursor(8, 1);
				lcd.print(batteries[lastScreen].voltage);
				lcd.setCursor(12, 1);
				lcd.print("V");
				break;
			}case EBatteryState::Charging:
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
			}case EBatteryState::Discharging:
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
				lcd.print(round(batteries[lastScreen].resistance));
				lcd.setCursor(9, 1);
				lcd.print("m");
				lcd.setCursor(10, 1);
				lcd.write(0);
				lcd.setCursor(12, 1);
				if ((lcdTimeOut - LCD_TIMEOUT/2) > millis())
				{
					lcd.print(batteries[lastScreen].loadVoltage);
					lcd.setCursor(15, 1);
					lcd.print("A");
				}
				else
				{
					lcd.print(round(batteries[lastScreen].temperature));
					lcd.setCursor(14, 1);
					lcd.write(1);
				}
				break;
			}case EBatteryState::Recharging:
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
			}case EBatteryState::Finished:
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
				lcd.print(round(batteries[lastScreen].resistance));
				lcd.setCursor(13, 1);
				lcd.print("m");
				lcd.setCursor(14, 1);
				lcd.write(0);
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
}


