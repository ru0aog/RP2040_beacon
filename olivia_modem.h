#ifndef OLIVIA_MODEM_H  
#define OLIVIA_MODEM_H  
  
#include <Arduino.h>  
  
// Подготовка сетки из 8 тонов с шагом 31.25 Гц (для PIO-VFO/Si5351)  
void prepare_olivia_frequencies(uint32_t base_hz);  
  
// Публичный API модема Olivia 8/250  
void olivia_send_char(char c);              // внутреннее — добавление символа в блок  
void olivia_send_string(const char* str);   // передача C-строки  
void olivia_send_string(String str);        // перегрузка для String  
  
#endif // OLIVIA_MODEM_H