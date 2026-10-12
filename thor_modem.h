#ifndef THOR_MODEM_H  
#define THOR_MODEM_H  
  
#include <Arduino.h>  
#include "si5351_driver.h"  
  
// Выбор подрежима: 8 или 11  
#define THOR_MODE 8          // 8 -> THOR 8, 11 -> THOR 11  
  
extern uint32_t THOR_Base_freq;  
extern uint8_t  DATA_THOR[18][8];   // регистровые пакеты для Si5351 (18 тонов)  
  
void prepare_thor_frequencies(uint32_t base_hz);  
void thor_send_char(char c);                 // один ASCII-символ через всю цепочку  
void thor_send_string(const char* str);      // передача C-строки  
void thor_send_string(String str);           // перегрузка для String  
  
#endif // THOR_MODEM_H