/**  
 * ============================================================================  
 *  vfo_hardware.h — Гибридный PWM/MASH-2 двигатель КВ VFO (Core 1)
 *  Версия 5.00 (Исправленная параметризация делителей КВ), 2026-10-03
 * ----------------------------------------------------------------------------  
 *  НАЗНАЧЕНИЕ  
 *  Заголовок описывает low-level API управления ВЧ-генератором на базе ШИМ.
 *  
 *  ПРИНЦИП КАСКАДНОЙ ГЕНЕРАЦИИ  
 *  1) vfo_find_optimal_pll() подбирает clk_sys так, чтобы минимизировать остаток
 *     дробного делителя на целевой базовой частоте.
 *  2) Аппаратный ШИМ работает в инверсном КВ-режиме: WRAP зафиксирован на минимуме
 *     (2..4), а весь основной коэффициент деления перенесен в регистр CH_DIV (8.4).
 *     Это сжимает цену младшего бита FRAC (1/16) до минимума, сужая девиацию MASH.
 *  3) Программный MASH-2 на Core 1 берет на себя тонкую доводку частоты до ~1 Гц,
 *     модулируя регистр CH_DIV на частоте в несколько мегагерц.
 *  
 *  РАСПРЕДЕЛЕНИЕ СПЕКТРА  
 *  Благодаря перевернутой раскладке, мгновенный шаг девиации MASH минимален.
 *  Горб шума квантования (NTF) отодвинут на частоту работы Core 1 (мегагерцы),
 *  что гарантирует идеальную чистоту ближней зоны (Гц..кГц) и легкую фильтрацию
 *  внешним аналоговым ФНЧ.
 * ============================================================================  
 */

#ifndef VFO_HARDWARE_H
#define VFO_HARDWARE_H

#include <Arduino.h>

// Компиляционные переключатели движка
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

#define VFO_CALIBRATED_XOSC_HZ 12000350ULL

struct VfoParameters {
    uint32_t pwm_wrap;        // Регистр TOP (период ШИМ)
    uint32_t pwm_base_div_fx4;// Базовый делитель (INT<<4 | FRAC) в формате 8.4
    uint32_t dds_step;        // Остаток шага для 32-битного программного MASH-2
    uint32_t target_freq_mhz; // Целевая частота в миллигерцах
};

// Прецизионное размещение таблицы частот в RAM (Синхронизировано с .cpp)
extern VfoParameters __attribute__((section(".time_critical.ifkp_tones"))) ifkp_tones[VFO_IFKP_TONES_COUNT];

// Low-Level API
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz);
void vfo_set_tone_instant(uint8_t tone_index);
void vfo_operation_set(bool key_down);          

#endif // VFO_HARDWARE_H
