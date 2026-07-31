#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h> // Подключаем вашу штатную библиотеку часов

// Аппаратная конфигурация пинов
const uint8_t DS_POWER_PIN = 16;
const uint8_t DS_PIN_SDA   = 14;
const uint8_t DS_PIN_SCL   = 15;

// Делаем объект часов видимым для всех модулей
extern bool DS_FAIL;
extern RTC_DS3231 rtc;

// Прототипы функций планировщика маяка
void init_scheduler();
void update_scheduler();
void print_current_time();
void print_current_date();
String get_telemetry_string();

extern uint8_t  rtc_hour;
extern uint8_t  rtc_min;
extern uint8_t  rtc_sec;
extern uint16_t rtc_year;
extern uint8_t  rtc_month;
extern uint8_t  rtc_day;

extern String rtc_chip_name; // будем пользоваться переменной из основного INO-файла

void I2C_DS_restart(); // переключить шину Wire1 на устройство DS3231
void handle_time_command(String cmd);
void handle_date_command(String cmd);
bool is_time_to_transmit(uint8_t mode);


#endif
