/**  
 * ============================================================================  
 *  vfo_hardware.h — Публичный интерфейс и конфигурация программного VFO  
 *  Версия 2.10 (Профилирование и ASM-оптимизация Core 1), 2026-09-26  
 * ============================================================================  
 */

#ifndef VFO_HARDWARE_H
#define VFO_HARDWARE_H

#include <Arduino.h>

// === Конфигурация механизмов снижения спуров ===
#define VFO_USE_MASH2            // Включить Delta-Sigma 2-го порядка (MASH-1-1). Если выключено — 1-й порядок.
#define VFO_DITHER_RANDOMIZE     // Включить рандомизацию входа аккумулятора (Dither Injection)
#define VFO_DITHER_RAND_BITS   4 // Амплитуда рандомизации в младших битах (4 бита: шум в диапазоне -7..+7)
/*
| N | размах дизера |
| 1 | всегда 0 (дизер фактически выключен) |
| 2 | −1…+1 |
| 3 | −3…+3 |
| 4 | −7…+7 |
| 5 | −15…+15 |
| 6 | −31…+31 |
*/
#define VFO_SNAP_TO_GRID         // Включить привязку частоты в окне +-0.1 Гц для минимизации полной дроби

// === Оптимизация и профилирование цикла дизеринга (Новое в v2.30) ===
#define VFO_DITHER_FAST          // Включить локализацию переменных, __builtin_add_overflow и Inline ASM
//#define VFO_DITHER_PROFILE       // Вывод меандра частоты цикла на отладочный GPIO_PROFILE_PIN
#define VFO_PROFILE_PIN      13  // Свободный пин для замера F_s_dither (половина частоты на осциллографе)

// === Динамический автотюнинг PLL ===
#define VFO_PLL_AUTOTUNE         // Адаптивный подбор clk_sys под целевую частоту для CW/IFKP/RTTY

// === ЧАСТЬ A: Двухъядерный режим ===
#define VFO_DITHER_ON_CORE1      // Вынос дизеринга в плотный цикл на Core 1.

// === ЧАСТЬ B: Фиксированная тактовая частота (Используется, ТОЛЬКО если выключен VFO_PLL_AUTOTUNE) ===
#define VFO_CLOCK_133_MHZ        // Разгон clk_sys до 133 МГц; иначе 120 МГц.

// === Настройка интервала таймера (используется только если VFO_DITHER_ON_CORE1 выключен) ===
#define VFO_DITHER_INTERVAL_US 10 

// === Аппаратная конфигурация физического уровня ===
#define VFO_OUTPUT_PIN       14            // Сигнал строго на GPIO 28
#define VFO_IFKP_TONES_COUNT 33            // Количество фиксированных тонов в сетке
#define VFO_TONE_NONE        255           // Флаг неопределенного тона

// Истинная физическая частота опорного кварца вашего экземпляра платы (калибровка)
#define VFO_CALIBRATED_XOSC_HZ 12000350ULL

// Структура параметров частоты для низкоуровневых регистров PIO и таймера фазы
struct VfoParameters {
    uint32_t pio_int;
    uint32_t pio_frac;
    uint32_t dds_step;
    uint32_t target_freq_chz; // Частота в сантигерцах (0.01 Гц)
};

extern VfoParameters ifkp_tones[VFO_IFKP_TONES_COUNT];

// === НАБОР ФУНКЦИЙ УПРАВЛЕНИЯ ВЧ-ЭФИРОМ (Low-Level API) ===
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz);
void vfo_set_tone_instant(uint8_t tone_index);
void vfo_set_cw_key(bool key_down);

#endif // VFO_HARDWARE_H
