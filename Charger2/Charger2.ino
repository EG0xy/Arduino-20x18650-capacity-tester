#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <CD74HC4067.h>
#include <Wire.h>
#include <Adafruit_PWM_Servo_Driver_Library\Adafruit_PWMServoDriver.h>
/* Connection
A0 - LCD Keypad shield - 5 keys array
~0 - Right
~130 - Up
~306 - Down
~479 - Left
~719 - Select

A1 - Multiplexer HP4067 - 1 (SIG pin) Battery (16) + Voltage Reading
A2 - Multiplexer HP4067 - 2 (SIG pin) Battery (16) + Voltage Reading
A3 - Multiplexer HP4067 - 3 (SIG pin) Battery (16) + Voltage Reading
A4 - Multiplexer HC4067 - 4 (SIG pin) Battery (16) + Voltage Reading

D43 - Multiplexer HP4067 - 1 (EN pin) Battery (16) + Voltage Reading
D45 - Multiplexer HP4067 - 2 (EN pin) Battery (16) + Voltage Reading
D47 - Multiplexer HP4067 - 3 (EN pin) Battery (16) + Voltage Reading
D49 - Multiplexer HC4067 - 4 (EN pin) Battery (16) + Voltage Reading

D42 - Multiplexer HC4067 (S0 pin) Read control
D44 - Multiplexer HC4067 (S1 pin) Read control  <<--------------- EN pin should be set on multiplexer in use to LOW before seting read channel
D46 - Multiplexer HC4067 (S2 pin) Read control
D48 - Multiplexer HC4067 (S3 pin) Read control

2 x PCA9685PW Adresses in l2C: 0x40 and 0x41 
16x2 display Adress in l2C: 0x3F

0-15	MUX 1 
0-3		MUX 2 - Charged read out from Led R1 in TP4056 (NC)
4		MUX 2 - NC (not connected)
5-15	MUX 2
0-8		MUX 3 - Voltage read out from batteries
9-15	MUX 3
0-12	MUX 4 - Voltage read out from ressistor

81.85931902590525 - 0.11406781021018966x + 0.00007701200364055409x2 - 2.666253770521e-8x3 + 3.54777456e-12x4 - formula for % diff from OPUS
*/
#define USE_SERIAL

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

//Button input
#define INPUT_PIN A0

#define BAT_COUNT 20
#define BAT_READ_COUNT 10
const float m_cfBatteryDischargeVoltage = 4.1;//V
const float m_cfBatteryCutOffVoltage = 2.85;//V
const float m_cfDischargeCurrent = 1000;//mA
static int currentMUX = 0;

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address for a 16 chars and 2 line display
//Multiplexer control
CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);

Adafruit_PWMServoDriver pwmDriver[PWM_DRIVER_COUNT] = {Adafruit_PWMServoDriver(0x40),Adafruit_PWMServoDriver(0x41)};


enum class EBatteryState :int
{
	Empty = 0,//Voltage visible if lower then m_cfBatteryDischargeVoltage
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

struct Batterie
{
	Data batterieReadPinsData;
	Data ressistorReadPinsData;
	Data pwmPinsData;
	EBatteryState state;
	unsigned int pwm;
	float capacity;
	float resistance;
	float voltage;
	float loadVoltage;
	float dischargeCurrent;
	unsigned long lasTime;

	Batterie() :state(EBatteryState::Empty), pwm(0), capacity(0.f), resistance(0.f), voltage(0.f), loadVoltage(0.f), dischargeCurrent(m_cfDischargeCurrent), lasTime(0) {};
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
	analogRead(muxSig[pinData.id]);//to reduce errors for bad reads skip first read
	for (uint8_t i = 0; i < BAT_READ_COUNT; i++) {
		sample += analogRead(muxSig[pinData.id]);
	}
	sample = sample / BAT_READ_COUNT;

	//voltage is read from voltage divider so multiplication by 2 is necesary
	return (float)sample * voltageMultiplierValue * (5.0 / 1023.0);
}

void UpdateBatteries()
{
	//load voltage and batterie voltage reads in separate loops in order to minimize the multiplexer switching
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].voltage = getVoltage(batteries[i].batterieReadPinsData, 2.f);
	}
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i].loadVoltage = getVoltage(batteries[i].ressistorReadPinsData);
	}
}
void UpdateCapacity(float current,int batID)
{
	unsigned long nDeltaTime = millis() - batteries[batID].lasTime;
	batteries[batID].capacity += current * (float)nDeltaTime / 3600000.f;
	batteries[batID].lasTime += nDeltaTime;
}

void SetPWM(float current, int batID) {
	bool bChanged = false;
	if (current < batteries[batID].dischargeCurrent && batteries[batID].pwm < MAX_PWM) {//for 8bit=255, for 16bit=65535
		batteries[batID].pwm += 1;
		bChanged = true;
	}
	else if (current > batteries[batID].dischargeCurrent && batteries[batID].pwm > 0) {
		batteries[batID].pwm -= 1;
		bChanged = true;
	}

	if(bChanged)
		pwmDriver[batteries[batID].pwmPinsData.id].setPWM(batteries[batID].pwmPinsData.chanel, 0, batteries[batID].pwm);
}

void Update(int batid)
{
	if (batteries[batid].state == EBatteryState::Discharging)
	{
		UpdateCapacity(batteries[batid].loadVoltage * 1000.f, batid);
	}

	SetPWM(batteries[batid].loadVoltage * 1000.f, batid);
#ifdef USE_SERIAL
	Serial.print(batteries[batid].loadVoltage);
	Serial.print(" ");
#endif // use_serial
}

void setup() {
	Serial.begin(9600);
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

		batteries[i].pwm = 4000;

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
		batteries[i].ressistorReadPinsData.id = pwmPICNum;
	}

	lcd.setBacklight(LOW);

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
}

void loop() {
	UpdateBatteries();

	for (int i = 0; i < BAT_COUNT; ++i)
	{
		Update(i);
	}
#ifdef USE_SERIAL
	Serial.println();
#endif
}
