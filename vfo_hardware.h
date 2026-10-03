/**  
 * ============================================================================  
 *  vfo_hardware.h — Гибридный PWM/MASH-2 двигатель КВ VFO (Core 1)
 *  Версия 4.00 (Дизеринг по CH_DIV PWM регистра вместо PIO), 2026-10-03
 * ============================================================================  
 */

#ifndef VFO_HARDWARE_H
#define VFO_HARDWARE_H

#include <Arduino.h>

// Ключевые конфигурационные макросы
#define VFO_USE_MASH2            // Включить Delta-Sigma 2-го порядка (MASH-1-1)
#define VFO_DITHER_RANDOMIZE     // Включить декорреляцию псевдослучайным дизером
#define VFO_DITHER_RAND_BITS   4 // Глубина дизера (-7..+7) в младших битах
#define VFO_PLL_AUTOTUNE         // Адаптивный автотюн clk_sys под базовый тон
#define VFO_DITHER_FAST          // Высокоскоростной Си-конвейер Core 1
#define VFO_DITHER_ON_CORE1      // Вынос горячего цикла на изолированное ядро

extern int pin_freq_out; 
#define VFO_OUTPUT_PIN       (pin_freq_out)
#define VFO_IFKP_TONES_COUNT 33            
#define VFO_TONE_NONE        255           

// Калиброванная частота кварца вашей платы
#define VFO_CALIBRATED_XOSC_HZ 12000350ULL

// Структура параметров под гибридную схему модуляции PWM
struct VfoParameters {
    uint32_t pwm_wrap;        // Регистр TOP (период ШИМ, защелкнут аппаратно)
    uint32_t pwm_base_div_fx4;// Базовый делитель (INT<<4 | FRAC) в формате 8.4
    uint32_t dds_step;        // Остаток шага для 32-битного программного MASH-2
    uint32_t target_freq_mhz; // Целевая частота в миллигерцах (для контроля)
};

extern VfoParameters ifkp_tones[VFO_IFKP_TONES_COUNT];

// API управления
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz);
void vfo_set_tone_instant(uint8_t tone_index);
void vfo_operation_set(bool key_down);          

#endif // VFO_HARDWARE_H
