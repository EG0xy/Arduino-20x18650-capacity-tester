#pragma once

#include <LiquidCrystal_I2C.h>

#define LCD_TIMEOUT 10000
#define LCD_REFRESH 500

LiquidCrystal_I2C lcd(0x3f, 16, 2);
