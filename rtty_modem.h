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
