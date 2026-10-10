#include <Arduino.h>
#include "vfo_hardware.h"
#include "olivia_modem.h"

// Константы Olivia 8/250
#define OLIVIA_TONES        8
#define OLIVIA_BAUD         31.25f
#define OLIVIA_SYMBOL_MS    32       // 1000 / 31.25 = 32 мс на один тон
#define INTERLEAVER_LEN     4        // Длина задержки перемежителя для полосы 250 Гц

// Матрица Уолша-Адамара (64 вектора по 64 бита) для кодирования 6 бит данных
// Вектор определяет последовательность из 64 символов (0 или 1)
static const uint64_t walsh_table[64] = {
    0x0000000000000000ULL, 0xFFFFFFFFFFFFFFFFULL, 0x00000000FFFFFFFFULL, 0xFFFFFFFF00000000ULL,
    0x0000FFFF0000FFFFULL, 0xFFFF0000FFFF0000ULL, 0x0000FFFFFFFF0000ULL, 0xFFFF00000000FFFFULL,
    0x00FF00FF00FF00FFULL, 0xFF00FF00FF00FF00ULL, 0x00FF00FFFF00FF00ULL, 0xFF00FF0000FF00FFULL,
    0x00FFFF0000FFFF00ULL, 0xFFFF0000FFFF0000ULL, 0x00FFFF00FFFF00BBULL, 0xFFFF00000000FFFFULL,
    0x0F0F0F0F0F0F0F0FULL, 0xF0F0F0F0F0F0F0F0ULL, 0x0F0F0F0FF0F0F0F0ULL, 0xF0F0F0F00F0F0F0FULL,
    0x0F0FF0F00F0FF0F0ULL, 0xF0F00F0FF0F00F0FULL, 0x0F0FF0F0F0F00F0FULL, 0xF0F00F0F0F0F0F0FULL,
    0x0FFFF00F0FFFF00FULL, 0xF0000FF0F0000FF0ULL, 0x0FFFF00FF0000FF0ULL, 0xF0000FF00FFFF00FULL,
    0x0FF00FF00FF00FF0ULL, 0xF00FF00FF00FF00FULL, 0x0FF00FF0F00FF00FULL, 0xF00FF00F00FF00FFULL,
    0x3333333333333333ULL, 0xCCCCCCCCCCCCCCCCULL, 0x33333333CCCCCCCCULL, 0xCCCCCCCC33333333ULL,
    0x3333CCCC3333CCCCULL, 0xCCCC3333CCCC3333ULL, 0x3333CCCCCCCC3333ULL, 0xCCCC33333333CCCCULL,
    0x33CC33CC33CC33CCULL, 0xCC33CC33CC33CC33ULL, 0x33CC33CCCC33CC33ULL, 0xCC33CC3333CC33CCULL,
    0x33CCCC3333CCCC33ULL, 0xCCCC3333CCCC3333ULL, 0x33CCCC33CCCC3333ULL, 0xCCCC33333333CCCCULL,
    0x3C3C3C3C3C3C3C3CULL, 0xC3C3C3C3C3C3C3C3ULL, 0x3C3C3C3CC3C3C3C3ULL, 0xC3C3C3C33C3C3C3CULL,
    0x3C3CC3C33C3CC3C3ULL, 0xC3C33C3CC3C33C3CULL, 0x3C3CC3C3C3C33C3CULL, 0xC3C33C3C3C3C3C3CULL,
    0x3CC33CC33CC33CC3ULL, 0xC33CC33CC33CC33CULL, 0x3CC33CC3C33CC33CULL, 0xC33CC33C3CC33CC3ULL,
    0x3CC3C33C3CC33CC3ULL, 0xC33CC33CC33CC33CULL, 0x3CC3C33CC33CC33CULL, 0xC33CC33C3C3C3C3CULL
};

// Буфер циклического перемежителя (Interleaver) для Olivia 8 тонов
// Нам нужно 3 бита (так как тонов 8 = 2^3) задерживать в массиве разной длины
static uint8_t interleaver_buffer[OLIVIA_TONES][INTERLEAVER_LEN];
static uint32_t interleaver_ptr = 0;

// Скремблер-генератор (M-последовательность) для защиты от белых пятен в спектре
static uint32_t olivia_scrambler_state = 0x1FFu;
static inline uint8_t get_scrambler_bit() {
    uint32_t bit = ((olivia_scrambler_state >> 0) ^ (uint32_t)(olivia_scrambler_state >> 4)) & 1u;
    olivia_scrambler_state = (olivia_scrambler_state >> 1) | (bit << 8);
    return (uint8_t)bit;
}

/**
 * @brief Передает один готовый 64-битный вектор Olivia в эфир
 * Разламывает вектор на 3-битные куски, прогоняет через перемежитель и шлет в Core 1.
 */
static void __not_in_flash_func(olivia_send_vector)(uint64_t walsh_vector) {
    // 64 бита вектора выдают 64 последовательных СВЧ-тона
    for (int i = 0; i < 64; i++) {
        uint32_t start_ms = millis();
        
        // Извлекаем текущий бит из вектора Уолша
        uint8_t walsh_bit = (walsh_vector >> (63 - i)) & 1ULL;
        
        // Накапливаем 3 бита из последовательных отсчетов для формирования индекса тона (0..7)
        // Для Olivia 8/250 берется 3 последовательных бита скремблированного потока
        uint8_t raw_tone_bit = walsh_bit ^ get_scrambler_bit();
        
        // Простейший циклический сдвиг перемежителя по каноническому алгоритму Olivia
        // Каждый из 3-х битов тона задерживается на разное число символов
        uint8_t bit_index = i % 3; 
        interleaver_buffer[bit_index][interleaver_ptr] = raw_tone_bit;
        
        // Считываем задержанный бит со смещением (компенсация замираний в эфире)
        uint32_t delay_idx = (interleaver_ptr + bit_index + 1) % INTERLEAVER_LEN;
        uint8_t delayed_bit = interleaver_buffer[bit_index][delay_idx];
        
        // Собираем финальный 3-битный индекс тона (0..7) для текущей итерации
        static uint8_t tone_accumulator = 0;
        tone_accumulator = (tone_accumulator << 1) | delayed_bit;
        
        if (bit_index == 2) {
            uint8_t final_tone = tone_accumulator & 0x07u;
            
            // МГНОВЕННАЯ ОТПРАВКА ТОНА В ГОРЯЧЕЕ ЯДРО CORE 1 БЕЗ РАЗРЫВА ФАЗЫ!
            vfo_set_tone_instant(final_tone);
            
            tone_accumulator = 0;
            
            // Жесткий рантайм-тайминг: удерживаем тон ровно 32 миллисекунды (31.25 Бод)
            while ((millis() - start_ms) < OLIVIA_SYMBOL_MS) {
                watchdog_update(); // Пинаем общую собаку
            }
        }
    }
    interleaver_ptr = (interleaver_ptr + 1) % INTERLEAVER_LEN;
}

/**
 * @brief Кодирует одиночный символ ASCII (7 бит) в формат Olivia
 */
void olivia_send_char(char c) {
    // В протоколе Olivia ASCII символ раскладывается на 6-битный индекс вектора Уолша
    // (Младшие 6 бит символа кодируют основной вектор, старший 7-й бит управляет инверсией)
    uint8_t walsh_idx = c & 0x3Fu;
    uint8_t invert = (c >> 6) & 1u;
    
    uint64_t vector = walsh_table[walsh_idx];
    if (invert) {
        vector = ~vector; // Инвертируем маску для расширения алфавита
    }
    
    // Выталкиваем 64-символьный блок в эфир
    olivia_send_vector(vector);
}

/**
 * @brief Передача текстовой строки по протоколу Olivia 8/250
 */
void olivia_send_string(const char* str) {
    // Включаем ключ трансивера (нажатие несущей)
    vfo_operation_set(true);
    
    // Передаем несколько символов синхронизации (символ 'П' или пробелы по спецификации)
    for(int i = 0; i < 4; i++) {
        olivia_send_char(' '); 
    }
    
    // Посимвольный асинхронный поток передачи
    while (*str) {
        olivia_send_char(*str++);
    }
    
    // Выключаем передатчик
    vfo_operation_set(false);
}
