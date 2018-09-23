#ifndef PWM_H
#define PWM_H

#include "Mux.h"
#include "Battery.h"

#define PWM_DRIVER_COUNT 2
#define PWM_CH_COUNT 16

#define MAX_PWM 3300//4095
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
		for (size_t ch = 0; ch < PWM_CH_COUNT; ch++)
		{
			pwmDriver[i].setPWM(ch, 0, PWM_OFF);
		}
	}
};

void setPWM(Battery& bat,int _pwm = -1)
{
	pwmDriver[bat.pwmDriverData.deviceId].setPWM(bat.pwmDriverData.channel, 0, max((_pwm < 0? bat.pwm:_pwm), MAX_PWM));
};

#endif //PWM_H
