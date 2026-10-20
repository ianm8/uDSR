#pragma once

#include <Wire.h>
#include <Tiny4kOLED_common.h>

static bool oledPresent = false;

static void tiny4koled_begin_wire1(void)
{
  Wire1.begin();
  Wire1.beginTransmission(SSD1306);
  oledPresent = (Wire1.endTransmission() == 0);
  // 0 = ACK received
}

static const bool oledFound(void)
{
  return oledPresent;
}

static bool tiny4koled_beginTransmission_wire1(void)
{
  Wire1.beginTransmission(SSD1306);
  return true;
}

static bool datacute_write_wire1(uint8_t byte)
{
  return Wire1.write(byte);
}

static uint8_t datacute_endTransmission_wire1(void)
{
  return Wire1.endTransmission();
}

SSD1306PrintDevice oled(
  &tiny4koled_begin_wire1,
  &tiny4koled_beginTransmission_wire1,
  &datacute_write_wire1,
  &datacute_endTransmission_wire1);