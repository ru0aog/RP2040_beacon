#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>
#include <Wire.h>

// Адрес часов на шине I2C
#define RTC_I2C_ADDRESS 0x68

// Прототипы функций планировщика маяка
void init_scheduler();
void update_scheduler();
void print_current_time();
void print_current_date();
String get_telemetry_string();
String get_current_time();
String get_current_date();

extern uint8_t  rtc_hour;
extern uint8_t  rtc_min;
extern uint8_t  rtc_sec;
extern uint16_t rtc_year;
extern uint8_t  rtc_month;
extern uint8_t  rtc_day;

extern uint8_t device_DS[5];

extern String rtc_chip_name; // будем пользоваться переменной из основного INO-файла

void I2C_DS_restart(); // переключить шину Wire1 на устройство DS3231
void handle_time_command(String cmd);
void handle_date_command(String cmd);
bool is_time_to_transmit(uint8_t mode);


#endif
