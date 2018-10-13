/*
	Name:       20x.ino
	Created:	2018-09-28 00:21:12
	Author:     Egidijus Griguola
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CD74HC4067.h>
#include <LiquidCrystal_I2C.h>

//#define USE_SERIAL//_INFO
//#define DEBUG
#define DEBUG_SELECT
static int timer = 30000;


#ifdef DEBUG
static int bt = 19;
static bool bPWMTogle = false;
static bool bChargeTogle = false;
#else
#define SCROLL_TIME 2000
static bool bTogleScroll = false;
static long long scrollTimer = SCROLL_TIME;
#endif // DEBUG
const float m_cfBatteryCutOffVoltage = 2.8f;//V
const float m_cfBatteryDischargeVoltage = 4.1;

#define INTERNAL1_1_REF 1.066
#define INTERNAL2_56_REF 2.496
#define INTERNAL5_REF 4.96

#define THERMAL_COOLDOWN 120000//2min

struct Data {
	int chanel;
	int id;

	int	iValue;
	float fValue;

	Data() :chanel(0), id(0), iValue(0), fValue(0.f) {};
};

struct RData {
	short index = 0;
	float voltage[2] = { 0.f,0.f };
	float current[2] = { 0.f,0.f };
	int	iValue = 0;
	unsigned long timer = 0;
};

#define RES_VALUE 1.f//1.1f
#define RESISTANCE_TIMER 3000 //waits for millliseconds until another mesurement for resistance calculation
#define PWM_DELAY 2000
#define PWM_DRIVER_COUNT 2
#define PWM_CH_COUNT 16
#define PWM_CHARGE 0
#define PWM_ON 3135 // ~1000mA
#define MAX_PWM 4095
#define PWM_OFF 1523
#define PWM_I_1 2306 // ~300mA
#define PWM_I_2 3135 // ~1000mA

Adafruit_PWMServoDriver pwmDriver[PWM_DRIVER_COUNT] = { Adafruit_PWMServoDriver(0x40),Adafruit_PWMServoDriver(0x41) };

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address for a 16 chars and 2 line display

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

CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);

uint8_t muxSig[MUX_COUNT] = { MUX_1_SIG, MUX_2_SIG, MUX_3_SIG, MUX_4_SIG };
uint8_t muxEN[MUX_COUNT] = { MUX_1_EN, MUX_2_EN, MUX_3_EN, MUX_4_EN };

//Button input
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

enum class EBatteryState :int
{
	Empty = 0,
	Low,//batteries always skips this state if DISABLE_CHARGING undefined (precharging enabled)
	Charging, //charging from TP4056 
	Resistance,
	Discharging,
	Recharging, //to 3.7V
	Finished//results
};

EKeyID activeButton = EKeyID::None;
bool InputActive = false;
static unsigned int nFinishedCount = 0;

#define LCD_TIMEOUT 10000
#define LCD_REFRESH 500

short lastScreen = 0;
unsigned long lcdTimeOut = LCD_TIMEOUT;
unsigned long screenRefresh = LCD_REFRESH;
unsigned long batteryLastInfoTimer = 0;
bool bLastInfoTrigered = false;

#define BAT_COUNT 20

struct SBattery
{
	Data voltageData;
	Data loadData;
	Data pwmData;
	Data thermalData;

	RData resistanceData;

	EBatteryState state;

	unsigned long pwmDelayCycles;
	float dischargeCurrent; //this is for if you need to set seperate discharge rates on slots
	unsigned long lastTimeStamp;
	float capacity;

	void Reset()
	{
		resistanceData.voltage[0] = 0.f;
		resistanceData.voltage[1] = 0.f;
		resistanceData.current[0] = 0;
		resistanceData.current[1] = 0;
		resistanceData.iValue = 0;
		resistanceData.index = 0;
		resistanceData.timer = 0;

		voltageData.fValue = 0.f;
		loadData.fValue = 0.f;
		pwmData.iValue = PWM_OFF;
		thermalData.fValue = 0.f;
		capacity = 0.f;
		lastTimeStamp = millis();
		dischargeCurrent = 1000.f;
		pwmDelayCycles = 0.f;
		state = EBatteryState::Empty;

		pwmDriver[pwmData.id].setPWM(pwmData.chanel, 0, pwmData.iValue);
	}

};

// Define User Types below here or use a .h file
//


// Define Function Prototypes that use User Types below here or use a .h file
//


// Define Functions below here or use other .ino or cpp files
//

static SBattery batteries[BAT_COUNT];

void clearReadings(byte port = A0)
{
	for (size_t i = 0; i < 5; i++)
	{
		analogRead(port);
		delay(1);
	}
}

// The setup() function runs once each time the micro-controller starts
void setup()
{
	analogReference(DEFAULT);
	clearReadings();

	Wire.begin();
	Serial.begin(115200);

	lcd.begin(16, 2);
	//	UpdateDisplay();
	lcd.setBacklight(LOW);

	timer += millis();

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
		delay(1000);
		for (size_t ch = 0; ch < PWM_CH_COUNT - (PWM_CH_COUNT - 4) * i; ch++)
		{
#ifdef USE_SERIAL
			Serial.print("PWM : ");
			Serial.print(i + 1);
			Serial.print(" - ");
			Serial.print(ch + 1);
			Serial.println();
#endif
			pwmDriver[i].setPWM(ch, 0, PWM_OFF);
		}
	}

	delay(2000);

	int muxTEMPNum = 0;
	int muxCH_TEMPNum = 0;
	int muxBVNum = 1;
	int muxCH_BVNum = 5;
	int muxRVNum = 2;
	int muxCH_RVNum = 9;
	int pwmPICNum = 0;
	int pwmCHNum = 0;

	for (size_t i = 0; i < BAT_COUNT; i++)
	{
		batteries[i] = SBattery();

		if (muxCH_TEMPNum >= MUX_CH_COUNT)
		{
			muxCH_TEMPNum = 0;
			++muxTEMPNum;
		}

		batteries[i].thermalData.chanel = muxCH_TEMPNum++;
		batteries[i].thermalData.id = muxTEMPNum;

		if (muxCH_BVNum >= MUX_CH_COUNT)
		{
			muxCH_BVNum = 0;
			++muxBVNum;
		}

		batteries[i].voltageData.chanel = muxCH_BVNum++;
		batteries[i].voltageData.id = muxBVNum;

		if (muxCH_RVNum >= MUX_CH_COUNT)
		{
			muxCH_RVNum = 0;
			++muxRVNum;
		}

		batteries[i].loadData.chanel = muxCH_RVNum++;
		batteries[i].loadData.id = muxRVNum;

		if (pwmCHNum >= PWM_CH_COUNT)
		{
			pwmCHNum = 0;
			++pwmPICNum;
		}

		batteries[i].pwmData.chanel = pwmCHNum++;
		batteries[i].pwmData.id = pwmPICNum;

		batteries[i].Reset();
	}

}

const float voltage_multiplier = 1.995f;

float voltageReadCalibration[BAT_COUNT] =
{
	1.0076093516 * voltage_multiplier,
	1.0076093516 * voltage_multiplier,
	1.0101804598 * voltage_multiplier,
	1.0153622422 * voltage_multiplier,
	0.9975062992 * voltage_multiplier,
	1.0050776036 * voltage_multiplier,
	1.0050776036 * voltage_multiplier,
	1.0076093516 * voltage_multiplier,
	1.0076093516 * voltage_multiplier,
	1.0076093516 * voltage_multiplier,
	1.0024800292 * voltage_multiplier,
	1.0076093516 * voltage_multiplier,
	1.0024538596 * voltage_multiplier,
	1.0179461351 * voltage_multiplier,
	1.0179461351 * voltage_multiplier,
	1.0101804598 * voltage_multiplier,
	1.0076093516 * voltage_multiplier,
	1.0025323726 * voltage_multiplier,
	1.0025323726 * voltage_multiplier,
	1.0			 * voltage_multiplier
};

constexpr float c_VoltageParts = 1.f / 1023.f;

static int currentMUX = 0;

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
	clearReadings(muxSig[pinData.id]);

	size_t count = 0;

	for (size_t i = 0; i < 10; i++)
	{
		float fRead = analogRead(muxSig[pinData.id]);

		if (fRead <= 0.05f)//do not use low values
		{
			++count;
			if (count > 5) { sample = 0.f; break; }
			--i;
			continue;
		}
		sample += fRead;
	}

	return (float)(sample * 0.1f) * voltageMultiplierValue * INTERNAL5_REF * c_VoltageParts;
}

float thermalCorrection[BAT_COUNT] =
{
	-6.6901,
	-6.9994,
	-6.4844,
	-7.7241,
	-6.8962,
	-6.9994,
	-6.8962,
	-7.6203,
	-6.5873,
	-6.8962,
	-20.2027,
	-7.1027,
	-6.4844,
	-7.7241,
	-7.413,
	-7.413,
	-7.3094,
	-7.828,
	-7.206,
	-7.6203
	/*0.7494935854,
	0.7390146471,
	0.7520325203,
	0.7238343658,
	0.7467204844,
	0.744217231	,
	0.7494935854,
	0.7288246881,
	0.7545887152,
	0.7520325203,
	0.5123471036,
	0.7520325203,
	0.7600136939,
	0.7390146471,
	0.744217231	,
	0.744217231	,
	0.7467204844,
	0.7338842975,
	0.7494935854,
	0.7390146471*/
};

float getCellTemp(const Data& pinData, int t = 1)
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
	clearReadings(muxSig[pinData.id]);

	float R1 = 10000;
	float logR2, R2, T, Tc, Tf;
	float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

	R2 = R1 * (1023.0 / (float)analogRead(muxSig[pinData.id]) - 1.0);
	logR2 = log(R2);
	T = (1.0 / (c1 + c2 * logR2 + c3 * logR2*logR2*logR2));
	return (float)(T - 273.15);// added -2 to get close to temperature, but formula should be adopted to thermistor instead!!
}

void SetPWM(int batID, int setCurrent = 0) {

	if (batteries[batID].voltageData.fValue < m_cfBatteryCutOffVoltage)
	{
		if (batteries[batID].pwmData.iValue != PWM_OFF)
		{
			batteries[batID].pwmData.iValue = PWM_OFF;
			pwmDriver[batteries[batID].pwmData.id].setPWM(batteries[batID].pwmData.chanel, 0, PWM_OFF);
		}
		return;
	}

	int current = batteries[batID].loadData.fValue * 1000.f;// / RES_VALUE;

	if (setCurrent == 0)
		setCurrent = batteries[batID].dischargeCurrent;

	//Serial.print("SETING PWM ");
	if (setCurrent > 0 && setCurrent != current && batteries[batID].pwmDelayCycles <= millis())
	{
		//Serial.print("ADJUSTING ++++++++++++++++++++++++\t");
		batteries[batID].pwmDelayCycles = millis() + PWM_DELAY;
		int newPwm = batteries[batID].pwmData.iValue;
		//for faster current equality
		int currentDiff = setCurrent - current;
		int pwmChange = map((int)(abs(currentDiff) *  0.2f), 0, 1600, 0, MAX_PWM - 1850);
#ifdef USE_SERIAL_INFO
		Serial.print("Dif: ");
		Serial.print(currentDiff);
		Serial.print("\t");
#endif
		if (currentDiff > 0 && newPwm < MAX_PWM)
		{
			newPwm += max(pwmChange, 0);// 
		}
		else if (currentDiff < 0 && newPwm > 0)
		{
			newPwm -= max(pwmChange, 0);// 
		}

		if (newPwm > MAX_PWM)
			newPwm = MAX_PWM;
		if (newPwm < PWM_OFF)
			newPwm = PWM_OFF;

		if (batteries[batID].pwmData.iValue != newPwm)
		{
			batteries[batID].pwmData.iValue = newPwm;
			pwmDriver[batteries[batID].pwmData.id].setPWM(batteries[batID].pwmData.chanel, 0, batteries[batID].pwmData.iValue);
		}
		return;
	}
}

void UpdateCapacity(int nID)
{
	unsigned long nDeltaTime = millis() - batteries[nID].lastTimeStamp;
	float current = batteries[nID].loadData.fValue;// / RES_VALUE;
	batteries[nID].capacity += (current * 1000.f) * (float)nDeltaTime / 3600000.f;
	batteries[nID].lastTimeStamp += nDeltaTime;
}

void ResetLastInfo()
{
	if (bLastInfoTrigered)
	{
		bLastInfoTrigered = false;
		batteries[lastScreen].Reset();
		batteryLastInfoTimer = millis();
	}
}

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
	EBatteryState searchState = EBatteryState::Finished;
#ifdef DEBUG_SELECT
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

		if (!bFound && nCount == BAT_COUNT && searchState > EBatteryState::Low)
		{
			searchState = (EBatteryState)((int)searchState - 1);
#ifdef DEBUG_SELECT
			Serial.print("Priority Selecttion loop: switch to ");
			Serial.print((int)searchState);
			Serial.print(" state");
			Serial.println();
#endif
			nCount = 0;
		}

	} while (!bFound && nCount <= BAT_COUNT);

	if (!bFound)
		result = 0;

#ifdef DEBUG_SELECT
	Serial.print("Priority Selecttion loop: returning ");
	Serial.print(result);
	Serial.println();
#endif

	return result;
}

void UpdateState()
{
	nFinishedCount = 0;
	for (size_t i = 0; i < BAT_COUNT; ++i)
	{
		bool reinserted = bLastInfoTrigered &&
			lastScreen == i &&
			batteries[i].state != EBatteryState::Finished &&
			batteries[i].state != EBatteryState::Charging &&
			batteryLastInfoTimer > millis() &&
			batteries[lastScreen].voltageData.fValue > m_cfBatteryCutOffVoltage;

		if (reinserted)
		{
			bLastInfoTrigered = false;
			batteryLastInfoTimer = millis() + LCD_TIMEOUT / 2;
		}

		if (!reinserted && batteries[i].state != EBatteryState::Empty && batteries[i].voltageData.fValue <= m_cfBatteryCutOffVoltage - 0.5f)
		{
			if (!bLastInfoTrigered)
			{
				//EnableLCD();
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
						batteryLastInfoTimer = millis() + LCD_TIMEOUT;
						lastScreen = PrioritySelect(1, lastScreen);

						//if (lastScreen >= 0)
						//{
						//	batteryLastInfoTimer = millis() + LCD_TIMEOUT / 2;
						//	//EnableLCD(0.5f);
						//}
					}
				}
				else
				{
					//if (lastScreen >= 0)
					batteries[lastScreen].Reset();
					batteryLastInfoTimer = millis() + LCD_TIMEOUT;
					lastScreen = i;
					//EnableLCD();
				}
			}
			continue;
		}

		switch (batteries[i].state)
		{
		case EBatteryState::Empty:
		{

#ifndef DISABLE_CHARGING
			if (batteries[i].voltageData.fValue >= m_cfBatteryCutOffVoltage) {
				batteries[i].state = EBatteryState::Charging;

				//using this to delay the charge switching for correct voltage readings of TP 4056
				batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;

				if (!bLastInfoTrigered)
				{
					//EnableLCD();
					lastScreen = i;
				}
			}
			else
#else
			if (batteries[i].voltageData.fValue < m_cfBatteryDischargeVoltage)
			{
				batteries[i].state = EBatteryState::Low;
			}
			else
#endif
				if (batteries[i].voltageData.fValue >= m_cfBatteryDischargeVoltage)
				{
					batteries[i].state = EBatteryState::Charging;// Resistance;
					batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;
					//break;
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
				if (batteries[i].voltageData.fValue >= m_cfBatteryDischargeVoltage)// if you add another battery in to holder
				{
					batteries[i].state = EBatteryState::Charging;// Resistance;
					batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;
					//EnableLCD();
					if (!bLastInfoTrigered)
					{
						//EnableLCD();
						lastScreen = i;
					}
				}
				else
				{
#ifdef USE_SERIAL
					Serial.print("Batterie : ");
					Serial.print(i + 1);
					Serial.print(" LOW: ");
					Serial.print(batteries[i].voltageData.fValue);
					Serial.print(" V");
					Serial.print(" Load: ");
					Serial.print(batteries[i].loadData.fValue);// / RES_VALUE);
					Serial.print(" V");
					Serial.print(" PWM: ");
					Serial.print(batteries[i].pwmData.iValue);
					Serial.println();
#endif
					break;
				}
			}
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

			if (batteries[i].state == EBatteryState::Charging)
			{
				if (batteries[i].resistanceData.timer > millis())
				{
					if (batteries[i].pwmData.iValue != PWM_OFF)
					{
						batteries[i].pwmData.iValue = PWM_OFF;
						pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
					}
					break;
				}
				else
				{
					batteries[i].resistanceData.timer = 0;
				}

				if (batteries[i].pwmData.iValue != PWM_CHARGE)
				{
					batteries[i].pwmData.iValue = PWM_CHARGE;
					pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
					break;
				}
				else if (batteries[i].thermalData.fValue < 0.f) // ~-50 will be set when TP4056 STDBY activates
				{
					batteries[i].thermalData.fValue = 20.f;
					batteries[i].pwmData.iValue = PWM_OFF;
					pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
					batteries[i].state = EBatteryState::Resistance;
				}
				else
				{
					if (batteries[i].thermalData.fValue >= 40.f)
					{
						batteries[i].resistanceData.timer = millis() + THERMAL_COOLDOWN;//2 min cool down 120000
					}
					break;
				}
			}
#else
			batteries[i].state = EBatteryState::Resistance;
#endif
		}
		case EBatteryState::Resistance:
		{
			if (batteries[i].resistanceData.timer <= 0)
			{
				batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;

				//if (!bLastInfoTrigered)
				//{
				//	//EnableLCD();
				//	lastScreen = i;
				//}

				pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, PWM_I_1);
			}
			else if (batteries[i].resistanceData.iValue == 0 && batteries[i].resistanceData.timer <= millis())
			{
				batteries[i].resistanceData.voltage[batteries[i].resistanceData.index] = batteries[i].voltageData.fValue;
				batteries[i].resistanceData.current[batteries[i].resistanceData.index] = batteries[i].loadData.fValue;// / RES_VALUE;

				++batteries[i].resistanceData.index;

				if (batteries[i].resistanceData.index >= 2)
				{
					//IEC 61960-2003
					batteries[i].resistanceData.iValue = (int)(1000 * fabsf(batteries[i].resistanceData.voltage[1] - batteries[i].resistanceData.voltage[0]) / fabsf(batteries[i].resistanceData.current[1] - batteries[i].resistanceData.current[0]));

#ifdef USE_SERIAL
					Serial.print("Batterie : ");
					Serial.print(i + 1);
					Serial.print(" resistance : ");
					Serial.print(batteries[i].resistanceData.iValue);
					Serial.print(" mOhm (IEC 61960-2003)");
					Serial.println();
					Serial.print("  V-1 : ");
					Serial.print(batteries[i].resistanceData.voltage[0]);
					Serial.print("  I-1 : ");
					Serial.print(batteries[i].resistanceData.current[0]);
					Serial.println();
					Serial.print("  V-2 : ");
					Serial.print(batteries[i].resistanceData.voltage[1]);
					Serial.print("  I-2 : ");
					Serial.print(batteries[i].resistanceData.current[1]);
					Serial.println();
#endif
				}
				else
				{
					batteries[i].resistanceData.timer = millis() + RESISTANCE_TIMER;
					pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, PWM_I_2);
				}
			}

			batteries[i].state = EBatteryState::Discharging;
		}
		case EBatteryState::Discharging:
		{

			if (batteries[i].resistanceData.index >= 2 && batteries[i].resistanceData.timer > millis())
			{
				if (batteries[i].pwmData.iValue != PWM_OFF)
				{
					batteries[i].pwmData.iValue = PWM_OFF;
					pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
				}
				break;
			}

			if (batteries[i].voltageData.fValue <= m_cfBatteryCutOffVoltage)// && (lastScreen == i?!bLastInfoTrigered:true))
			{
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" DISCHARGE completed");
				Serial.println();
#endif
				batteries[i].pwmData.iValue = PWM_OFF;
				pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
				batteries[i].state = EBatteryState::Recharging;
#ifdef DISABLE_CHARGING
				batteries[i].voltageData.fValue = m_cfRechargeVoltage;
#endif
			}
			else
			{
				if (batteries[i].pwmData.iValue == PWM_CHARGE || batteries[i].pwmData.iValue == PWM_OFF)
				{
					batteries[i].pwmData.iValue = map((int)(batteries[i].dischargeCurrent), 0, 1600, PWM_OFF + 1, MAX_PWM);
					batteries[i].lastTimeStamp = millis();
					if (batteries[i].resistanceData.index >= 2)//Do not change PWM while resistance test running
						SetPWM(i); //pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
					batteries[i].state = EBatteryState::Discharging;
					break;
				}

				if (batteries[i].resistanceData.index < 2 && batteries[i].resistanceData.timer <= millis())
				{
					batteries[i].state = EBatteryState::Resistance;
				}

				UpdateCapacity(i);
				if (batteries[i].resistanceData.index >= 2)//Do not change PWM while resistance test running
					SetPWM(i);

				if (batteries[i].thermalData.fValue >= 40.f)
				{
					batteries[i].resistanceData.timer = millis() + THERMAL_COOLDOWN;//2 min cool down 120000
				}
#ifdef USE_SERIAL
				Serial.print("Batterie : ");
				Serial.print(i + 1);
				Serial.print(" Capacity: ");
				Serial.print(batteries[i].capacity);
				Serial.print(" --- V : ");
				Serial.print(batteries[i].voltageData.fValue);
				Serial.print(" --- A : ");
				Serial.print(batteries[i].loadData.fValue);// / RES_VALUE);
				Serial.print(" --- PWM : ");
				Serial.print(batteries[i].pwmData.iValue);
				Serial.println();
#endif
				break;
			}
		}
		case EBatteryState::Recharging:
		{
#ifdef USE_RECHARGE
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
			if (!bLastInfoTrigered)// && lastScreen <= LCD_ON)//Finished screen is shown
			{
				//EnableLCD();
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

		if (batteries[i].state == EBatteryState::Finished)
			++nFinishedCount;
	}
}

void UpdateBatterie()
{
#ifdef USE_SERIAL_INFO
	Serial.print("**********************************************************");
	Serial.println();
#endif
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].thermalData.fValue = getCellTemp(batteries[i].thermalData) + thermalCorrection[i];
#ifdef USE_SERIAL_INFO
		Serial.print(batteries[i].thermalData.fValue, 2);
		//Serial.print((char)0xDF);
		Serial.print("  C\t");
#endif
	}
#ifdef USE_SERIAL_INFO
	Serial.println();
#endif
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].voltageData.fValue = /*(batteries[i].thermalData.fValue < 0 && batteries[i].state != EBatteryState::Charging) ? 0.f :*/ getVoltage(batteries[i].voltageData, voltageReadCalibration[i]);
#ifdef USE_SERIAL_INFO
		Serial.print(batteries[i].voltageData.fValue, 4);
		Serial.print(" V");
		Serial.print("\t");

#endif
	}
#ifdef USE_SERIAL_INFO
	Serial.println();
#endif
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].loadData.fValue = getVoltage(batteries[i].loadData, 1.0153f);
#ifdef USE_SERIAL_INFO
		/*Serial.print("L: ");
		Serial.print(batteries[i].loadData.fValue, 4);
		Serial.print("\t");*/

		float current = batteries[i].loadData.fValue;// / RES_VALUE;
		Serial.print(current, 4);
		Serial.print(" A");
		Serial.print("\t");
#endif
	}
}
void UpdateDisplay()
{
#ifndef DEBUG
	if (bTogleScroll && !bLastInfoTrigered && scrollTimer <= millis())
	{
		scrollTimer = millis() + SCROLL_TIME;
		lastScreen = PrioritySelect(1, lastScreen, nFinishedCount > 0);
	}
#endif
	if (screenRefresh >= millis()) return;

	if (/*lcdTimeOut > 0 &&*/ screenRefresh < millis())
	{
		screenRefresh = millis() + LCD_REFRESH;
	}

	//if (lastScreen == LCD_ON) // MAIN screen
	//{
	//	lastScreen = LCD_OFF; // To prevent clearing screen repeatedly
	//	lcd.clear();
	//	lcd.setCursor(1, 0);
	//	lcd.print("Battery Tester");
	//	lcd.setCursor(5, 1);
	//	lcd.print("Ver 2");
	//	//EnableLCD();
	//}
	//else
	if (timer > millis())
	{
		lcd.clear();
		lcd.setCursor(2, 1);
		lcd.print("INITIALIZING");
		return;
	}

	if (lastScreen >= 0)// && lcdTimeOut > millis()) // Display the last active battery for some time then reset to MAIN screen
	{
		if (batteries[lastScreen].resistanceData.timer > millis() && batteries[lastScreen].resistanceData.timer - millis() > RESISTANCE_TIMER)
		{
			lcd.clear();
#ifndef DEBUG
			if (bTogleScroll)
			{
				lcd.setCursor(0, 0);
				lcd.print("#");
			}
#endif
			lcd.setCursor(3, 0);
			lcd.print("Battery ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(4, 1);
			lcd.print("COOLING");
			return;
		}

		switch (batteries[lastScreen].state)
		{
		case EBatteryState::Empty:
		{
			lcd.clear();
#ifndef DEBUG
			if (bTogleScroll)
			{
				lcd.setCursor(0, 0);
				lcd.print("#");
			}
#endif
			lcd.setCursor(3, 0);
			lcd.print("Battery ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(1, 1);
			lcd.print("Empty  Slot!");
			break;
		}
		case EBatteryState::Low:
		{
			lcd.clear();
#ifndef DEBUG
			if (bTogleScroll)
			{
				lcd.setCursor(0, 0);
				lcd.print("#");
			}
#endif
			lcd.setCursor(3, 0);
			lcd.print("Battery ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(3, 1);
			lcd.print("LOW: ");
			lcd.setCursor(8, 1);
			lcd.print(batteries[lastScreen].voltageData.fValue);
			lcd.setCursor(12, 1);
			lcd.print("V");
			break;
		}
		case EBatteryState::Charging:
		{
			lcd.clear();
#ifndef DEBUG
			if (bTogleScroll)
			{
				lcd.setCursor(0, 0);
				lcd.print("#");
			}
#endif
			lcd.setCursor(3, 0);
			lcd.print("Battery ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(1, 1);
			lcd.print("Charging");
			lcd.setCursor(10, 1);
			lcd.print(batteries[lastScreen].voltageData.fValue);
			lcd.setCursor(14, 1);
			lcd.print("V");
			break;
		}case EBatteryState::Discharging:
		{
			lcd.clear();
#ifndef DEBUG
			if (bTogleScroll)
			{
				lcd.setCursor(0, 0);
				lcd.print("#");
			}
#endif
			lcd.setCursor(1, 0);
			lcd.print("Bat-");
			lcd.setCursor(5, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(8, 0);
			lcd.print(round(batteries[lastScreen].capacity));
			lcd.setCursor(12, 0);
			lcd.print("mAh");
			lcd.setCursor(0, 1);
			lcd.print(batteries[lastScreen].voltageData.fValue);
			lcd.setCursor(4, 1);
			lcd.print("V");
			lcd.setCursor(6, 1);
			lcd.print(batteries[lastScreen].resistanceData.iValue);
			lcd.setCursor(9, 1);
			lcd.print("mO");
			//lcd.setCursor(10, 1);
			//lcd.write(0);
			lcd.setCursor(12, 1);
			// TODO: make autoswitch for C and A
			if ((screenRefresh - LCD_REFRESH / 2) > millis())
			{
				lcd.print(batteries[lastScreen].loadData.fValue);// / RES_VALUE);
				lcd.setCursor(15, 1);
				lcd.print("A");
				
			}
			else
			{
				lcd.print((int)batteries[lastScreen].thermalData.fValue);
				lcd.setCursor(14, 1);
				lcd.print("C");
				//lcd.write(1);
			}
			break;
		}case EBatteryState::Recharging:
		{
			lcd.clear();
#ifndef DEBUG
			if (bTogleScroll)
			{
				lcd.setCursor(0, 0);
				lcd.print("#");
			}
#endif
			lcd.setCursor(3, 0);
			lcd.print("Battery ");
			lcd.setCursor(11, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(0, 1);
			lcd.print("ReCharging");
			lcd.setCursor(11, 1);
			lcd.print(batteries[lastScreen].voltageData.fValue);
			lcd.setCursor(15, 1);
			lcd.print("V");
			break;
		}case EBatteryState::Finished:
		{
			lcd.clear();
#ifndef DEBUG
			if (bTogleScroll)
			{
				lcd.setCursor(0, 0);
				lcd.print("#");
			}
#endif
			lcd.setCursor(3, 0);
			lcd.print("Batterie ");
			lcd.setCursor(12, 0);
			lcd.print(lastScreen + 1);
			lcd.setCursor(2, 1);
			lcd.print(round(batteries[lastScreen].capacity));
			lcd.setCursor(6, 1);
			lcd.print("mAh");
			lcd.setCursor(10, 1);
			lcd.print(batteries[lastScreen].resistanceData.iValue);
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
	//else if (lcdTimeOut <= millis() && lcdTimeOut > 0)
	//{
	//	lastScreen = 0;
	//	lcdTimeOut = 0;
	//	//lcd.clear();
	//	//lcd.setBacklight(HIGH);
	//}
}

// Add the main program code into the continuous loop() function
void loop()
{
	UpdateInput();
	UpdateBatterie();
#ifndef DEBUG
	if(timer < millis())
		UpdateState();

	UpdateDisplay();
#else
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		if (!bChargeTogle && bPWMTogle)
		{
			SetPWM(i);
		}
		else if (bChargeTogle && !bPWMTogle)
		{
			batteries[i].pwmData.iValue = batteries[i].thermalData.fValue < 0 || batteries[i].voltageData.fValue < m_cfBatteryCutOffVoltage ? PWM_OFF : PWM_CHARGE;
			pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
		}
		else
		{
			batteries[i].pwmData.iValue = PWM_OFF;
			pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
		}
	}
	lcd.setCursor(0, 0);
	if (bChargeTogle)
	{
		lcd.print("C: ");
	}
	else if (bPWMTogle)
	{
		lcd.print("L: ");
	}
	else
	{
		lcd.print(" : ");
	}
	lcd.setCursor(3, 0);
	lcd.print(batteries[bt].loadData.fValue);// / RES_VALUE);
	lcd.setCursor(7, 0);
	lcd.print("A");
	lcd.setCursor(9, 0);
	lcd.print(batteries[bt].thermalData.fValue);
	lcd.setCursor(14, 0);
	lcd.print((char)0xDF);
	lcd.print("C");

	lcd.setCursor(0, 1);
	lcd.print("    ");
	lcd.setCursor(0, 1);
	lcd.print(batteries[bt].pwmData.iValue);
	lcd.setCursor(5, 1);
	lcd.print(batteries[bt].voltageData.fValue);
	lcd.setCursor(10, 1);
	lcd.print("V");

	lcd.setCursor(12, 1);
	lcd.print("BT");
	lcd.setCursor(15, 1);
	lcd.print(" ");
	lcd.setCursor(14, 1);
	lcd.print(bt + 1);
#endif // !DEBUG

#ifdef USE_SERIAL_INFO
	Serial.println();
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		Serial.print(batteries[i].pwmData.iValue);
		Serial.print("\t");
	}
	Serial.println();
#endif
}

void UpdateInput()
{
	//Disable controlls for when battery is removed
	//if (bLastInfoTrigered && lastScreen > LCD_ON) return;

	clearReadings();

	int readkey = analogRead(A0);

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
	else if (readkey >= 800) {
		InputActive = false;
		activeButton = EKeyID::None;
#ifdef DEBUG
		Serial.print("All released");
		Serial.println();
#endif
	}

	if (activeButton == EKeyID::None)
		return;

	if (!InputActive)
	{
		switch (activeButton)
		{
		case EKeyID::Right:
		{
			InputActive = true;
#ifdef DEBUG
			for (int i = 0; i < BAT_COUNT; ++i)
			{
				batteries[i].pwmData.iValue += 50;
			}
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

			if (!bPWMTogle)
			{
				if (!bChargeTogle)
				{
					bChargeTogle = true;
				}
				else
				{
					bChargeTogle = false;
				}
			}
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
			//EnableLCD();//refreshing timer
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
			//EnableLCD();//refreshing timer
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

			if (!bChargeTogle)
			{
				if (!bPWMTogle)
				{
					for (int i = 0; i < BAT_COUNT; ++i)
					{
						batteries[i].pwmData.iValue = map((int)(batteries[i].dischargeCurrent), 0, 1600, PWM_OFF, MAX_PWM);
						pwmDriver[batteries[i].pwmData.id].setPWM(batteries[i].pwmData.chanel, 0, batteries[i].pwmData.iValue);
					}
					bPWMTogle = true;
				}
				else
				{
					bPWMTogle = false;
				}
			}
#else
			if (!bTogleScroll)
			{
				bTogleScroll = true;
			}
			else
			{
				bTogleScroll = false;
			}
#endif
			break;
		}
		default:
		{
		}

		};
	}
}
