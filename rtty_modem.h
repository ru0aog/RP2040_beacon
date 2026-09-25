/**  
 * ============================================================================  
 *  rtty_modem.h — Публичный интерфейс передатчика RTTY (МТК-2 / Бодо)  
 * ============================================================================  
 *  
 *  Объявляет типы и API модуля RTTY; реализация — в rtty_modem.cpp.  
 *  
 *  ТИПЫ:  
 *    enum TransmitterState { SPACE=0, MARK=1 } — состояние FSK-манипуляции:  
 *        SPACE — частота логического нуля (база), MARK — база + сдвиг.  
 *    enum MTK2_STATE { LAT, FIG, RUS } — регистр кода МТК-2 (латиница,  
 *        цифры/знаки, кириллица).  
 *    struct mtk2_map_t { const char* s; uint8_t code; } — элемент таблицы  
 *        соответствия «символ -> 5-битный код Бодо».  
 *  
 *  СКОРОСТЬ:  
 *    Стандарт RTTY 45.45 Бод -> длительность бита ~22000 мкс (задаётся  
 *    глобальной RTTY_BIT_TIME_US в основном скетче).  
 *  
 *  API:  
 *    prepare_rtty_frequencies(space_hz, mark_hz) — подготовка частот MARK/SPACE  
 *        (для Si5351 или программного VFO на PIO).  
 *    send_rtty_string(const char*) / send_rtty_string(String) — передача текста  
 *        с преамбулой и постамбулой (перегрузки для C-строк и объектов String).  
 * ============================================================================  
 */

#ifndef RTTY_MODEM_H
#define RTTY_MODEM_H

#include <Arduino.h>

// Скорость передачи RTTY (45.45 Бод -> время бита ~22000 мкс)
// const uint32_t BIT_TIME_US = 22000; 

// Перечисления для состояний передатчика RTTY
enum TransmitterState {
    SPACE = 0, // Частота логического нуля (обычно базовая)
    MARK  = 1  // Частота логической единицы (базовая + сдвиг)
};

// Перечисления регистров кода МТК-2
enum MTK2_STATE { 
    LAT, // Латинский
    FIG, // Цифровой/Знаки
    RUS  // Русский
};

// Структура карты символов МТК-2
struct mtk2_map_t {
    const char* s;
    uint8_t code;
};

// Прототипы функций модуля RTTY
void prepare_rtty_frequencies(uint32_t space_hz, uint32_t mark_hz);
void send_rtty_string(const char* s);
void send_rtty_string(String str); // Перегрузка для удобной отправки объектов String

#endif
