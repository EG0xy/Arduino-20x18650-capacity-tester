#ifndef BATTERY_H
#define BATTERY_H

#include "PWM_Drive.h"

enum State {
	Empty = 0,
	Paused, //Process pause for THERMAL_PAUSE
	Low,//batteries always skips this state if DISABLE_CHARGING undefined (precharging enabled)
	Charging, //charging from TP4056 
	Resistance,
	Discharging,
	Recharging, //to 3.7V
	Finished//results
};

enum ReadState {
	Voltage = 0,
	Load
};

struct ResistanceData {
	float voltage[2] = { 0.f,0.f };
	float current[2] = { 0.f,0.f };
	unsigned long timer;
	byte mesurementCount;
	float resistance;
};

struct Battery {
	ResistanceData resistanceData;

	ChannelData thermalReadData;
	ChannelData voltageReadData;
	ChannelData loadReadData;
	ChannelData pwmDriverData;

	State state;
	State lastState;

	float temperature;
	float capacity;
	float voltage, calibrateV;
	float loadVoltage;

	unsigned int pwm;
	float dischargeCurrent;

	unsigned long lastTime;

	//  0 - 15	MUX 0
	//	0 - 3	MUX 1 - Charged read out from Led R1 in TP4056(NC)
	//	4		MUX 1 - NC(not connected)
	//	5 - 15	MUX 1
	//	0 - 8	MUX 2 - Voltage read out from batteries
	//	9 - 15	MUX 2
	//	0 - 12	MUX 3 - Voltage read out from ressistor

	void init(float voltageCalibrate)
	{
		static int muxTEMPNum = 0;
		static int muxCH_TEMPNum = 0;
		static int muxBVNum = 1;
		static int muxCH_BVNum = 5;
		static int muxRVNum = 2;
		static int muxCH_RVNum = 9;
		static int pwmPICNum = 0;
		static int pwmCHNum = 0;

		calibrateV = voltageCalibrate;

		if (muxCH_TEMPNum >= MUX_CH_COUNT)
		{
			muxCH_TEMPNum = 0;
			++muxTEMPNum;
		}

		thermalReadData.channel = muxCH_TEMPNum++;
		thermalReadData.deviceId = muxTEMPNum;

		if (muxCH_BVNum >= MUX_CH_COUNT)
		{
			muxCH_BVNum = 0;
			++muxBVNum;
		}

		voltageReadData.channel = muxCH_BVNum++;
		voltageReadData.deviceId = muxBVNum;

		if (muxCH_RVNum >= MUX_CH_COUNT)
		{
			muxCH_RVNum = 0;
			++muxRVNum;
		}

		loadReadData.channel = muxCH_RVNum++;
		loadReadData.deviceId = muxRVNum;

		if (pwmCHNum >= PWM_CH_COUNT)
		{
			pwmCHNum = 0;
			++pwmPICNum;
		}

		pwmDriverData.channel = pwmCHNum++;

		pwmDriverData.deviceId = pwmPICNum;

		pwm = PWM_OFF;

		setPWM(pwmDriverData, pwm);		
	};

	void Reset()
	{
		resistanceData.voltage[0] = 0.f;
		resistanceData.voltage[1] = 0.f;
		resistanceData.current[0] = 0.f;
		resistanceData.current[1] = 0.f;
		resistanceData.mesurementCount = 0;
		resistanceData.timer = 0;
		state = State::Empty;
		lastState = State::Empty;
		pwm = PWM_OFF;
		temperature = 0;
		capacity = 0.f;
		voltage = 0.f;
		loadVoltage = 0.f;
		resistanceData.resistance = 0.f;
		lastTime = 0;

		setPWM(pwmDriverData, pwm);
	};

	float updateVoltage(ReadState readState)
	{
		const ChannelData& data = (readState == Voltage ? voltageReadData : loadReadData);
		setMux(data);

		float sample = 0.f;

		for (uint8_t i = 0; i < BAT_READ_COUNT; i++)
		{
			sample += (float)analogRead(muxSig[data.deviceId]);
		}
		sample = sample / (float)BAT_READ_COUNT;

		float reading = (float)sample *((readVcc() * 0.001f) / 1023.0);

		if (readState == Voltage)
		{
			voltage = reading * 2.f;
			voltage += voltage * calibrateV;
		}
		else
		{
			loadVoltage = reading;
			loadVoltage += loadVoltage * RESISTANCE_MULTIPLIER;
		}
	};

	void updateTemp()
	{
		setMux(thermalReadData);

		float sample = 0.f;
		float R1 = 10000.f;
		float logR2, R2, T, Tc, Tf;
		//float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
		float c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741;

		R2 = R1 * (1023.f / (float)analogRead(muxSig[thermalReadData.deviceId]) - 1.0);
		logR2 = log(R2);
		T = (1.0 / (c1 + c2 * logR2 + c3 * logR2*logR2*logR2));
		temperature = (float)(T - 273.15);// -(thermalReadData.channel == 10 ? 14 : 0));// added -2 to get close to temperature, but formula should be adopted to thermistor instead!!
	};

	void UpdateCapacity()
	{
		unsigned long nDeltaTime = millis() - lastTime;
		capacity += loadVoltage * 1000.f * (float)nDeltaTime / 3600.f;
		lastTime += nDeltaTime;
	};

	void adjustPWM(float _current = -1.f) {
		if (_current < 0)
			_current = loadVoltage * 1000.f;

		if (_current > 0 && _current != dischargeCurrent)
		{
			int newPwm = pwm;
			//for faster current equality
			int currentDiff = round(dischargeCurrent - _current);
			//int pwmChange = map((int)(abs(currentDiff) *  0.2f), 0, 2000, 0, MAX_PWM);

			if (currentDiff > 0 && newPwm < MAX_PWM)
			{
				newPwm += 1;// max(pwmChange, 1);
			}
			else if (currentDiff < 0 && newPwm > 0)
			{
				newPwm -= 1;// max(pwmChange, 1);
			}

			if (newPwm > MAX_PWM)
				newPwm = MAX_PWM;
			if (newPwm < PWM_OFF)
				newPwm = PWM_OFF;

			if (pwm != newPwm)
			{
				pwm = newPwm;
				setPWM(pwmDriverData, pwm);
			}
		}
	};
};
#endif; //BATTERY_H
