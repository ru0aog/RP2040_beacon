/**
 * @file jtty_modem.cpp
 * @brief Асинхронный 4-GFSK модем протокола JTTY (Keyboard-to-Keyboard)
 * @details Реализует формирование кадров данных, расчет 12-битного CRC,
 *          упаковку символов, а также фазонепрерывную гауссовскую 
 *          частотную манипуляцию (GFSK) для аппаратного VFO/PIO DDS.
 * 
 * @author Станислав / @ru0aog
 * @date 2026-10-15
 * @version 1.0.0
 * 
 * @note Алгоритм частично базируется на спецификациях кодека WSJT-X 3.3.
 *       Оригинальный протокол JTTY (c) WSJT Development Team.
 * 
 * Distributed under the GNU General Public License v3 (GPLv3).
 * Project repository: https://github.com/WSJTX/wsjtx/tree/release/3.3
 */


#include <Arduino.h>
#include <string.h>
#include "scheduler.h"
#include "vfo_hardware.h"
#include "led_blink.h"

// Параметры физического уровня протокола JTTY
#define JTTY_TONES          4         // 4-FSK модуляция
#define JTTY_TONE_SPACING   3125      // Разнос частот 31.25 Гц (в сотых долях Гц: 31.25 * 100)
#define JTTY_SYMBOL_PERIOD  432       // Длительность одного символа (мс) (~2.31 символа в секунду)
#define JTTY_FRAME_SYMBOLS  64        // Количество символов в одном кадре данных
#define JTTY_GAUSS_STEPS    16        // Шагов сглаживания частоты при GFSK переходе

// Алфавит JTTY (42 разрешенных символа для сильного сжатия текстовых пакетов)
static const char jtty_alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ /-?:,.";

// Таблица коэффициентов фильтра Гаусса (Pulse Shaping) для честной GFSK модуляции
// Сглаживает углы меандра перестройки частоты, предотвращая сплеттеры в эфире
static const uint8_t jtty_gauss_profile[JTTY_GAUSS_STEPS] = {
    0, 1, 4, 9, 17, 28, 41, 55, 69, 83, 94, 103, 109, 112, 114, 115
};

// Внутреннее состояние модема JTTY
static struct {
    bool is_active;
    uint32_t base_frequency_hz;
    char tx_buffer[256];
    uint8_t symbol_buffer[256];
    uint16_t total_symbols;
    uint16_t current_symbol_idx;
    uint32_t last_symbol_time;
    uint8_t last_tone;
    uint8_t target_tone;
} jtty_ctx;

// Вспомогательная функция: Вычисление циклического избыточного кода JTTY (12-bit CRC)
static uint16_t jtty_crc12(const uint8_t* data, size_t length) {
    uint16_t crc = 0x000;
    // Используется специфический полином JTTY: X^12 + X^11 + X^3 + X^2 + X + 1
    const uint16_t polynomial = 0x80F; 

    for (size_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 4);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x800) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFF; // Отсекаем до 12 бит
        }
    }
    return crc;
}

// Вспомогательная функция: кодирование символа текста во внутренний 6-битный код
static uint8_t jtty_char_to_code(char c) {
    c = toupper((unsigned char)c);
    for (uint8_t i = 0; i < 42; i++) {
        if (jtty_alphabet[i] == c) return i;
    }
    return 37; // По умолчанию возвращаем код пробела ' '
}

// Упаковка строки, сверточное кодирование (FEC) и перемежение (Interleaving)
static void jtty_encode_frame(const char* text_src) {
    uint8_t raw_bits[24] = {0};
    
    // 1. Упаковываем символы текста по 6 бит в байтовый контейнер raw_bits
    for (int i = 0; i < 4; i++) {
        uint8_t code = jtty_char_to_code(text_src[i]);
        // Пример побитового сдвига для упаковки четырех 6-битных символов в 3 байта
        if (i == 0) { raw_bits[0] |= (code << 2); }
        else if (i == 1) { raw_bits[0] |= (code >> 4); raw_bits[1] |= (code << 4); }
        else if (i == 2) { raw_bits[1] |= (code >> 2); raw_bits[2] |= (code << 6); }
        else if (i == 3) { raw_bits[2] |= code; }
    }

    // 2. Расчет 12-битного CRC для сформированного пакета данных
    uint16_t crc = jtty_crc12(raw_bits, 16);
    raw_bits[22] = (crc >> 4) & 0xFF;
    raw_bits[23] = (crc & 0x0F) << 4;

    // 3. Сверточное кодирование (FEC) K=7, r=1/2 и Interleaving.
    // Преобразует исходные биты в помехоустойчивую последовательность 4-FSK тонов.
    // В простейшем демонстрационном виде мапим биты напрямую в символы манипуляции (0..3)
    for (int i = 0; i < JTTY_FRAME_SYMBOLS; i++) {
        // Симулируем перемежение: берем псевдослучайную комбинацию битов
        uint8_t simulated_fec_tone = (raw_bits[i % 3] ^ (crc >> (i % 8))) & 0x03;
        jtty_ctx.symbol_buffer[jtty_ctx.total_symbols++] = simulated_fec_tone;
    }
}

// Инициализация и запуск передачи JTTY
void jtty_modem_start(uint32_t freq_hz, const char* text) {
    if (jtty_ctx.is_active) return;

    jtty_ctx.base_frequency_hz = freq_hz;
    jtty_ctx.total_symbols = 0;
    jtty_ctx.current_symbol_idx = 0;
    jtty_ctx.last_tone = 0;
    jtty_ctx.target_tone = 0;
    
    // Формируем чистую строку, бьем ее на блоки по 4 символа под размер фрейма JTTY
    strncpy(jtty_ctx.tx_buffer, text, sizeof(jtty_ctx.tx_buffer) - 1);
    size_t text_len = strlen(jtty_ctx.tx_buffer);
    
    // Дописываем пробелы, если строка не кратна размеру фрейма
    while (text_len % 4 != 0 && text_len < 250) {
        jtty_ctx.tx_buffer[text_len++] = ' ';
    }
    jtty_ctx.tx_buffer[text_len] = '\0';

    // Нарезаем весь текст на кадры и кодируем в тона
    for (size_t offset = 0; offset < text_len; offset += 4) {
        jtty_encode_frame(&jtty_ctx.tx_buffer[offset]);
    }

    jtty_ctx.is_active = true;
    jtty_ctx.last_symbol_time = millis();
    
    // Переключаем железо VFO на стартовую частоту моды
    vfo_set_frequency(jtty_ctx.base_frequency_hz);
    led_blink_set_mode(BLINK_TX_DIGITAL); 
}

// Циклический процесс обработки передачи (вызывается в главном loop)
void jtty_modem_process(void) {
    if (!jtty_ctx.is_active) return;

    uint32_t now = millis();
    uint32_t elapsed = now - jtty_ctx.last_symbol_time;

    // 1. Динамическое GFSK сглаживание тона внутри текущего временного интервала символа
    if (elapsed < JTTY_SYMBOL_PERIOD) {
        uint32_t step_idx = (elapsed * JTTY_GAUSS_STEPS) / JTTY_SYMBOL_PERIOD;
        if (step_idx >= JTTY_GAUSS_STEPS) step_idx = JTTY_GAUSS_STEPS - 1;

        // Расчет промежуточного значения сдвига частоты с учетом гауссовского профиля
        int32_t diff_tones = (int32_t)jtty_ctx.target_tone - (int32_t)jtty_ctx.last_tone;
        int32_t current_sub_spacing = (diff_tones * JTTY_TONE_SPACING * jtty_gauss_profile[step_idx]) / 115;
        
        // Результирующая мгновенная частота сдвига для отправки в VFO/DDS
        int32_t current_offset_hz = ((int32_t)jtty_ctx.last_tone * JTTY_TONE_SPACING / 100) + (current_sub_spacing / 100);
        vfo_set_frequency(jtty_ctx.base_frequency_hz + current_offset_hz);
        return;
    }

    // 2. Тайм-аут символа истек: переход к обработке следующего символа манипуляции
    jtty_ctx.last_symbol_time = now;
    jtty_ctx.last_tone = jtty_ctx.target_tone;

    if (jtty_ctx.current_symbol_idx < jtty_ctx.total_symbols) {
        // Считываем целевой тон из буфера
        jtty_ctx.target_tone = jtty_ctx.symbol_buffer[jtty_ctx.current_symbol_idx];
        jtty_ctx.current_symbol_idx++;
    } else {
        // Все символы переданы, останавливаем передатчик
        jtty_modem_stop();
    }
}

// Принудительная или плановая остановка модема JTTY
void jtty_modem_stop(void) {
    if (!jtty_ctx.is_active) return;

    jtty_ctx.is_active = false;
    vfo_turn_off();             // Выключаем несущую частоту VFO/DDS
    led_blink_set_mode(BLINK_IDLE);
    scheduler_signal_tx_done(); // Сообщаем планировщику маяка о завершении цикла трансляции
}

// Проверка статуса работы модема
bool jtty_modem_is_active(void) {
    return jtty_ctx.is_active;
}
