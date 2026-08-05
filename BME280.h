#ifndef BME280_H
#define BME280_H

#include <Arduino.h>
#include <Wire.h>

// Адрес датчика на шине I2C.
#define BME280_ADDRESS 0x76

const uint8_t BME_POWER_PIN = 11;
const uint8_t BME_PIN_SDA   = 12;
const uint8_t BME_PIN_SCL   = 13;

extern float  bme_humid;
extern float  bme_temp;
extern float  bme_press;

void init_BME();
void BME_read();
String get_climate_telemetry();

#endif