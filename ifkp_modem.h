/**  
 * ============================================================================  
 *  ifkp_modem.h — Публичный интерфейс передатчика IFKP  
 * ============================================================================  
 *  
 *  Объявляет глобальное состояние и API модуля IFKP; реализация — в  
 *  ifkp_modem.cpp.  
 *  
 *  ГЛОБАЛЬНОЕ СОСТОЯНИЕ (extern):  
 *    DATA_IFKP[33][8] — предрассчитанные 8-байтные регистровые пакеты 33 тонов  
 *                       для Si5351.  
 *    current_tone     — индекс текущего активного тона в сетке (0..32).  
 *    IFKP_Base_freq   — базовая (нижняя) частота сетки, Гц.  
 *    IFKP_STEP_HZ     — шаг сетки тонов, 11.697 Гц (= 386/33), общий для Si5351  
 *                       и программного VFO на PIO.  
 *  
 *  API:  
 *    prepare_ifkp_frequencies(base_hz)   — расчёт сетки тонов (Si5351) или  
 *                                          инициализация программного VFO на PIO.  
 *    send_ifkp_char(c)                   — модуляция одного символа (через дельту).  
 *    send_ifkp_string(const char*) / (String) — передача текста (UTF-8, с  
 *                                          транслитерацией кириллицы).  
 *    send_delta(delta)                   — инкрементальный сдвиг тона на дельту.  
 *    send_ifkp_calibration_ladder()      — калибровочный свип-тест 33 тонов.  
 * ============================================================================  
 */

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
extern const double IFKP_STEP_HZ; // Экспорт шага сетки частот IFKP для PIO
void send_ifkp_calibration_ladder(); // Передача тестовой лесенки тонов

#endif
