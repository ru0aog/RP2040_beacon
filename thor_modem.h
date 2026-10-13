#ifndef THOR_MODEM_H  
#define THOR_MODEM_H  
  
#include <Arduino.h>  
#include "si5351_driver.h"  
  
// Выбор подрежима THOR: 8 или 11  
#define THOR_MODE 11         // 8 -> THOR 8, 11 -> THOR 11  
  
extern uint32_t THOR_Base_freq;  
extern uint32_t MFSK16_Base_freq;
extern uint32_t DOMINOEX_Base_freq;
extern uint32_t FSQ_Base_freq;
extern uint8_t  DATA_MFSK[33][8];   // регистровые пакеты Si5351 текущей сетки (до 33 тонов)

// ---- Общий IFK/MFSK-движок ----
void mfsk_prepare(uint32_t base_hz, double step_hz, uint8_t numtones); // сетка тонов (Si5351/PIO)
bool mfsk_emit_tone(uint8_t tone, uint32_t symbol_us);                 // тон + дедлайн/LED/watchdog

// ---- RSID fldigi (15 символов GF16, 16 тонов 10.77 Гц, center — центр полосы) ----
void rsid_encode(uint16_t code, uint8_t* rsid /*15*/);
bool rsid_send(uint16_t code, double center_hz);  // меняет сетку: после вызова — mfsk_prepare()

// ---- THOR ----
void prepare_thor_frequencies(uint32_t base_hz);  
void thor_send_char(char c);                 // один ASCII-символ через всю цепочку  
void thor_send_string(const char* str);      // передача C-строки  
void thor_send_string(String str);           // перегрузка для String  

// ---- MFSK16: 16 тонов 15.625 Гц, 64 мс, RSID 57 ----
void prepare_mfsk16_frequencies(uint32_t base_hz);
void mfsk16_send_string(const char* str);
void mfsk16_send_string(String str);

// ---- DominoEX 8: 18 тонов 15.625 Гц, 128 мс, RSID 86 ----
void prepare_dominoex_frequencies(uint32_t base_hz);
void dominoex_send_string(const char* str);
void dominoex_send_string(String str);

// ---- FSQ-4.5: 33 тона 8.789 Гц, 256 мс, кадр " \n<CALL>:текст\n " ----
void prepare_fsq_frequencies(uint32_t base_hz);
void fsq_send_string(const char* str);
void fsq_send_string(String str);
  
#endif // THOR_MODEM_H
