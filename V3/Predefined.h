#pragma once

#define THERMAL_PAUSE 60000

#define BAT_COUNT 20
#define BAT_READ_COUNT 10

#define PWM_I_1 2127 // ~300mA
#define PWM_I_2 2950 // ~1000mA
#define PWM_IA_1 300 //mA
#define PWM_IA_2 1000 //mA

#define RESISTANCE_TIMER 3000 //waits for millliseconds until another mesurement for resistance calculation

#ifdef DISABLE_CHARGING
const float m_cfBatteryDischargeVoltage = 4.1;//V
#else
const float m_cfBatteryDischargeVoltage = 4.15f;//V
#endif // DISABLE_CHARGING
const float m_cfBatteryCutOffVoltage = 2.8f;//V
const float m_cfRechargeVoltage = 3.9f;//V
const float m_cfDischargeCurrent = 1000.f;//mA

static unsigned int nFinishedCount = 0;

#define RESISTANCE_MULTIPLIER 0.01056421f //1000mA mesured

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

	result = 1115070L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

	return result; // Vcc in millivolts
};
