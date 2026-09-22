#ifndef GEN_FRACT_H
#define GEN_FRACT_H

#include <Arduino.h>

// ==========================================
// КОНСТАНТЫ И НАСТРОЙКИ ОБОРУДОВАНИЯ
// ==========================================
const uint PIN_I = 28; // GPIO 28 - выход частоты

// Прототипы функций модуля
void set_pio_sdr_freq(uint32_t target_frequency_hz);
uint32_t fractGen_get_real_frequency();
void fractGen_init();
void fractGen_ON();
void fractGen_OFF();

#endif