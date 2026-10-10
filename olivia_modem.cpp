/**  
 * ============================================================================  
 *  olivia_modem.cpp — Модулятор Olivia 8/250 (8 тонов, полоса 250 Гц)  
 * ============================================================================  
 *  Структура блока: 3 ASCII-символа (по 7 бит) кодируются ПАРАЛЛЕЛЬНО, каждый  
 *  в свою бит-плоскость 3-битного MFSK-символа. Каждый символ раскладывается на  
 *  64-битный вектор Уолша-Адамара (6 бит -> номер строки, 7-й бит -> инверсия).  
 *  На выходе — 64 тона на блок, по 32 мс каждый (31.25 бод, шаг 31.25 Гц).  
 *  
 *  ВНИМАНИЕ: точная бит-в-бит совместимость с fldigi не гарантирована —  
 *  скремблер/перемежитель здесь самосогласованы, но могут отличаться от  
 *  эталонного алгоритма Jalocha. Проверять на реальном декодере.  
 * ============================================================================  
 */  
  
#include <Arduino.h>  
#include "olivia_modem.h"
#include "vfo_hardware.h"
#include "si5351_driver.h"
#include "led_blink.h"
#include <hardware/watchdog.h>  
  
// --- Внешние зависимости проекта (как в ifkp_modem.cpp) ---  
extern volatile bool pc_file_written;  
extern bool soft_restart_flag;  
extern void check_serial_commands();  
extern bool debug_flag;  
extern uint8_t device_SI[5];  
  
// --- Константы Olivia 8/250 ---  
#define OLIVIA_TONES      8  
#define OLIVIA_BITS       3        // log2(8) — бит на символ = число символов в блоке  
#define OLIVIA_SYMBOL_MS  32       // 1000 / 31.25 = 32 мс на тон  
static const double OLIVIA_STEP_HZ = 31.25; // шаг сетки = скорость = 250/8  
  
uint32_t OLIVIA_Base_freq = 3601307; // базовая частота по умолчанию  
  
// --- Матрица Уолша-Адамара 64x64, строится рекурсивно (без хардкода/опечаток) ---  
static uint64_t walsh_table[64];  
static bool     walsh_ready = false;  
  
static void olivia_build_walsh() {  
    // H[i][j] = (-1)^popcount(i & j); бит 1 ставим при нечётной чётности  
    for (int i = 0; i < 64; i++) {  
        uint64_t row = 0;  
        for (int j = 0; j < 64; j++) {  
            if (__builtin_parity((unsigned)(i & j)))  
                row |= (1ULL << (63 - j));  
        }  
        walsh_table[i] = row;  
    }  
    walsh_ready = true;  
}  
  
  
// --- Грей-код (убирает многократные ошибки при соседних тонах) ---  
static inline uint8_t to_gray(uint8_t v) { return v ^ (v >> 1); }  
  
// --- Накопитель блока: 3 символа ---  
static char    block_chars[OLIVIA_BITS];  
static uint8_t block_count = 0;  
  
// Подготовка сетки из 8 тонов (PIO-VFO или Si5351)  
void prepare_olivia_frequencies(uint32_t base_hz) {  
    OLIVIA_Base_freq = base_hz;  
    if (!walsh_ready) olivia_build_walsh();  
    block_count = 0;  
  
    if (!device_SI[0]) {  
        vfo_hardware_init(base_hz, OLIVIA_STEP_HZ);  
        vfo_set_tone_instant(0);  
        if (debug_flag) {  
            Serial.print(F("[OLIVIA] PIO готов, база "));  
            Serial.print(base_hz); Serial.print(F(" Гц, шаг "));  
            Serial.print(OLIVIA_STEP_HZ, 3); Serial.println(F(" Гц"));  
        }  
    }  
}  
  
// Удержание одного тона 32 мс с кормлением собаки и опросом CLI  
static inline bool olivia_emit_tone(uint8_t tone) {  
    vfo_set_tone_instant(tone);          // фазонепрерывный прыжок частоты  
    uint32_t start_ms = millis();  
    while ((millis() - start_ms) < OLIVIA_SYMBOL_MS) {  
        if (pc_file_written || soft_restart_flag) return false; // прерывание  
        check_serial_commands();  
        watchdog_update();  
        yield();  
    }  
    return true;  
}  
  
// Передача одного блока из 3 символов (64 тона)  
static bool olivia_flush_block() {  
    if (block_count == 0) return true;  
  
    // fldigi добивает блок НУЛЁМ (idle), а не пробелом  
    for (uint8_t p = block_count; p < OLIVIA_BITS; p++) block_chars[p] = 0;  
  
    const uint64_t SCRAMBLE = 0xE257E6D0291574ECULL; // ScramblingCodeOlivia  
    const uint8_t  NSHIFT   = 13;                     // шаг для Olivia  
  
    uint8_t out[64];  
    for (int t = 0; t < 64; t++) out[t] = 0;  
  
    // Для каждого из 3 символов (FreqBit) раскладываем строку Адамара,  
    // скремблируем и «размазываем» по бит-плоскостям (FreqBit + t) % 3  
    for (uint8_t freqBit = 0; freqBit < OLIVIA_BITS; freqBit++) {  
        uint8_t  c   = (uint8_t)block_chars[freqBit] & 0x7Fu;  
        uint8_t  idx = c & 0x3Fu;                 // 6 бит -> строка (позиция импульса)  
        uint8_t  inv = (c >> 6) & 1u;             // 7-й бит -> знак импульса (-1)  
        uint64_t row = inv ? ~walsh_table[idx] : walsh_table[idx]; // 1 = отрицательный отсчёт  
  
        // Скремблер: бит маски с индексом (freqBit*13 + t) & 63  
        uint8_t codeBit = (uint8_t)((freqBit * NSHIFT) & 63u);  
        for (int t = 0; t < 64; t++) {  
            uint8_t neg = (uint8_t)((row >> (63 - t)) & 1ULL);     // знак отсчёта FHT  
            neg ^= (uint8_t)((SCRAMBLE >> codeBit) & 1ULL);        // ScrambleFHT  
            if (neg) {                                             // FHT_Buffer[t] < 0  
                uint8_t bit = freqBit + (uint8_t)(t % OLIVIA_BITS);// (FreqBit + Rotate)  
                if (bit >= OLIVIA_BITS) bit -= OLIVIA_BITS;  
                out[t] |= (uint8_t)(1u << bit);  
            }  
            codeBit = (uint8_t)((codeBit + 1) & 63u);  
        }  
    }  
  
    // Выдаём 64 тона; модулятор fldigi применяет Грей-код к номеру тона  
    for (int t = 0; t < 64; t++) {  
        uint8_t tone = to_gray(out[t]) & (OLIVIA_TONES - 1);  
        if (!olivia_emit_tone(tone)) { block_count = 0; return false; }  
    }  
    block_count = 0;  
    return true;  
}
  
// Добавление символа в текущий блок; при заполнении — передача  
void olivia_send_char(char c) {  
    if (pc_file_written || soft_restart_flag) return;  
    if (c != '\r' && c != '\n') Serial.print(c);  
    block_chars[block_count++] = c;  
    if (block_count >= OLIVIA_BITS) olivia_flush_block();  
}  
  
// Передача строки по протоколу Olivia 8/250  
void olivia_send_string(const char* str) {  
    if (str == nullptr) return;  
    if (!walsh_ready) olivia_build_walsh();  
    block_count = 0;  
  
    // Поднимаем выход (PIO плавно, либо Si5351)  
    if (!device_SI[0]) {  
        vfo_operation_set(false);  
        vfo_set_tone_instant(0);  
        vfo_operation_set(true);  
    } else {  
        VFO_TX_ON();  
    }  
  
    // Преамбула — синхросимволы (пробелы)  
    for (int i = 0; i < OLIVIA_BITS; i++) olivia_send_char(' ');  
  
    // Полезная нагрузка  
    while (*str && !pc_file_written && !soft_restart_flag) {  
        olivia_send_char(*str++);  
    }  
  
    // Добиваем неполный последний блок  
    if (block_count > 0) olivia_flush_block();  
  
    // Закрываем сессию  
    if (device_SI[0]) VFO_TX_OFF();  
    else              vfo_operation_set(false);  
    ZERO_LED_OFF();  
    Serial.println("");  
}  
  
void olivia_send_string(String str) {  
    olivia_send_string(str.c_str());  
}