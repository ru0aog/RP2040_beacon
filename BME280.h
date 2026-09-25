/**  
 * ============================================================================  
 *  BME280.h — Публичный интерфейс драйвера климатических датчиков  
 * ============================================================================  
 *  
 *  Экспортирует три глобальных результата замера (bme_temp / bme_humid /  
 *  bme_press), конфигурацию датчика device_BM[5] (присутствие, № шины Wire,  
 *  пины SDA/SCL, I2C-адрес) и три функции API. Поддерживаются датчики на  
 *  адресах BME280_ADDRESS (0x76, BME280/BMP280) и BMP180_ADDRESS (0x77).  
 *  Реализация, автоопределение типа и формулы компенсации — в BME280.cpp.  
 *  
 *    init_BME()              — инициализация и автоопределение датчика.  
 *    BME_read()              — замер с выводом в Serial.  
 *    get_climate_telemetry() — строка климатической телеметрии.  
 * ============================================================================  
 */

#ifndef BME280_H
#define BME280_H

#include <Arduino.h>
#include <Wire.h>

// Адрес датчика на шине I2C.
#define BME280_ADDRESS 0x76
#define BMP180_ADDRESS 0x77

extern float  bme_humid;
extern float  bme_temp;
extern float  bme_press;

extern uint8_t device_BM[5]; // номер шины, пин SDA, пин SCL, адрес

void init_BME();
void BME_read();
String get_climate_telemetry();

#endif