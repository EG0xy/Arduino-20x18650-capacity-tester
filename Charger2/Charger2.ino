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
#define MUX_3_EN 46
#define MUX_4_EN 47

#define MAX_PWM 4095

//Button input
#define INPUT_PIN A0

#define BAT_COUNT 20
#define BAT_READ_COUNT 10
const float m_cfBatteryDischargeVoltage = 4.1;
const float m_cfBatteryCutOffVoltage = 2.85;

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address for a 16 chars and 2 line display
//Multiplexer control
CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);


enum class EBatteryState :int
{
	Empty = 0,
	PreCharge, //for TP4056 to charge up the batteries before disharging batteries need to be lower then 4V for TP4056 to start carging (Adding load until lower then 4V)
	Charging, //charging from TP4056 
	Resistance,
	Discharging,
	Recharging,
	Finished//results
};

struct Batterie
{
	EBatteryState state;
	unsigned int pwm;
	float capacity;
	float resistance;
	float voltage;
	float loadVoltage;
	unsigned long lasTime;

	Batterie() :state(EBatteryState::Empty), pwm(0), capacity(0.f), resistance(0.f), voltage(0.f), loadVoltage(0.f), lasTime(0) {};
};

Batterie batteries[BAT_COUNT];


//read voltage from analog pins
float getVoltage(uint8_t pin, float voltageDeviderValue = 1.0) {                     
	float sample = 0.0;
	analogRead(pin);//to reduce errors for bad reads skip first read
	for (uint8_t i = 0; i < BAT_READ_COUNT; i++) {
		sample += analogRead(pin);
	}
	sample = sample / BAT_READ_COUNT;

	//voltage is read from voltage divider so multiplication by 2 is necesary
	return (float)sample * voltageDeviderValue * (5.0 / 1023.0);
}

//void UpdateBatteryVoltage(int batID)//from 0
//{
//	float fVolts = 0.f;
//	
//	mux.channel(batID);
//	delay(1);
//	fVolts = getVoltage(MUX_SIG);
//
//	batteries[batID].voltage = fVolts * voltageCalibration[batID];
//}
//void UpdateCapacity(int batID)
//{
//	unsigned long nDeltaTime = millis() - batteries[batID].lasTime;
//	batteries[batID].capacity += currentCalibration[batID] * (float)nDeltaTime / 3600000.f;
//	batteries[batID].lasTime += nDeltaTime;
//}

//void Update(int batID)
//{
//	UpdateBatteryVoltage(batID);
//
//#ifdef USE_SERIAL
//	Serial.print(batteries[batID].voltage);
//	Serial.println();
//#endif // USE_SERIAL
//}

void setup() {
	for (int i = 0; i < BAT_COUNT; ++i)
	{
		batteries[i] = Batterie();
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

	//buttons
	pinMode(A0, INPUT);
	//***********************************************
	SPI.setBitOrder(LSBFIRST);
	// SPI_CLOCK_DIV2 is only a tiny bit faster in sending out the last byte.
	// SPI transfer and calculations overlap for the other bytes.
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.begin();
}

void loop() {

}
