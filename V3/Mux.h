#pragma once

#include <CD74HC4067.h>
#include <Wire.h>

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

static CD74HC4067 mux(MUX_S0, MUX_S1, MUX_S2, MUX_S3);

static byte currentMUX;

uint8_t muxSig[MUX_COUNT] = { MUX_1_SIG, MUX_2_SIG, MUX_3_SIG, MUX_4_SIG };
int muxEN[MUX_COUNT] = { MUX_1_EN, MUX_2_EN, MUX_3_EN, MUX_4_EN };

struct ChannelData {
	byte channel;
	byte deviceId;
};

void setupMux()
{
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
};

void setMux(const ChannelData& data, int t = 1)
{
	if (currentMUX != data.deviceId)
	{
		digitalWrite(muxEN[currentMUX], HIGH);
		digitalWrite(muxEN[data.deviceId], LOW);
		currentMUX = data.deviceId;
	}

	mux.channel(data.channel);
	delay(1);
};