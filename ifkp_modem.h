#ifndef IFKP_MODEM_H
#define IFKP_MODEM_H

#include <Arduino.h>
#include "si5351_driver.h" // Подключаем драйвер для работы с константами частот

extern uint8_t DATA_IFKP[33][8];
extern uint8_t current_tone;
extern uint32_t IFKP_Base_freq;

// Прототипы функций для работы маяка из основного скетча
void prepare_ifkp_frequencies(uint32_t base_hz);
void send_ifkp_char(char c);
void send_ifkp_string(const char* str);
void send_ifkp_string(String str);
void send_delta(uint8_t delta);

#endif
