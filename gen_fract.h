#ifndef GEN_FRACT_H
#define GEN_FRACT_H

#include <Arduino.h>

// ==========================================
// КОНСТАНТЫ И НАСТРОЙКИ ОБОРУДОВАНИЯ
// ==========================================
const uint PIN_I = 28; // GPIO 28 - Фаза I
const uint PIN_Q = 29; // GPIO 29 - Фаза Q

// Прототипы функций модуля
void set_pio_sdr_freq(uint32_t target_frequency_hz);
void fractGen_init();

#endif