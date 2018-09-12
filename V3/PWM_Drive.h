#pragma once

#include <Adafruit_PWMServoDriver.h>

#include "Mux.h"

#define PWM_DRIVER_COUNT 2
#define PWM_CH_COUNT 16

#define MAX_PWM 4095
#define PWM_ON 3115 //~1000mA
#define PWM_OFF 1023//NO charge AND NO DISCHARGE
#define PWM_CHARGE 0

static Adafruit_PWMServoDriver pwmDriver[PWM_DRIVER_COUNT] = { Adafruit_PWMServoDriver(0x40),Adafruit_PWMServoDriver(0x41) };

void setupPWM_Drives()
{
	for (size_t i = 0; i < PWM_DRIVER_COUNT; i++)
	{
		pwmDriver[i].begin();
		pwmDriver[i].setPWMFreq(1600);
	}
};

void setPWM(const ChannelData& data, unsigned int pwm)
{
	pwmDriver[data.deviceId].setPWM(data.channel, 0, pwm);
}