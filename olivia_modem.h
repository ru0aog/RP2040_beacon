#ifndef OLIVIA_MODEM_H  
#define OLIVIA_MODEM_H  
  
#include <Arduino.h>  
  
// Подготовка сетки из 8 тонов с шагом 31.25 Гц (для PIO-VFO/Si5351)  
void prepare_olivia_frequencies(uint32_t base_hz);  
void prepare_contestia_frequencies(uint32_t base_hz); // ДОБАВЛЕНО
  
// Публичный API модема Olivia 8/250  
void olivia_send_char(char c);              // внутреннее — добавление символа в блок  
void olivia_send_string(const char* str);   // передача C-строки (Olivia)  
void olivia_send_string(String str);        // перегрузка для String  
  
// Публичный API модема Contestia 8/250 (тот же движок, другой режим)  
void contestia_send_string(const char* str); // ДОБАВЛЕНО
void contestia_send_string(String str);      // ДОБАВЛЕНО
  
#endif // OLIVIA_MODEM_H