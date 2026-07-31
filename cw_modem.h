#ifndef CW_MODEM_H
#define CW_MODEM_H

#include <Arduino.h>

// Настройка скорости CW (длина точки в миллисекундах)
// меньше 20 не стоит делать
extern uint32_t CW_DOT_TIME_MS; 

void prepare_cw_frequency(uint32_t freq_hz);
void send_cw_string(const char* str);
void send_cw_string(String str);

#endif
