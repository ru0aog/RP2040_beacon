#ifndef SI5351_DRIVER_H
#define SI5351_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

const uint8_t SI_POWER_PIN = 17;
const uint8_t SI_PIN_SDA   = 18;
const uint8_t SI_PIN_SCL   = 19;

#define SI5351_I2C_ADDR    0x60

extern bool SI_FAIL;


// Прототипы функций модуля
void I2C_SI_restart(); // переключить шину Wire1 на устройство Si5351
void CLK_OFF_si5351(uint8_t CLK_NO); // выключить выход CLK_NO
void CLK_ON_si5351(uint8_t CLK_NO);  // включить  выход CLK_NO
void init_si5351_pins();
void setFrq_si5351(uint8_t SI_FREQ[], uint8_t CLK_NO);
void SI_POWER_ON();
void SI_POWER_OFF();
bool si5351_write_reg(uint8_t reg, uint8_t data);

#endif
