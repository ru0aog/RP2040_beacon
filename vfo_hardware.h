#ifndef VFO_HARDWARE_H
#define VFO_HARDWARE_H

#include <Arduino.h>

// === Аппаратная конфигурация физического уровня ===
#define VFO_OUTPUT_PIN       28            // Сигнал строго на GPIO 28 (Физический пин 34 платы)
#define VFO_IFKP_TONES_COUNT 33            // Количество фиксированных тонов в сетке
#define VFO_TONE_NONE        255           // Флаг неопределенного/сброшенного тона

// Истинная физическая частота опорного кварца вашего экземпляра платы (калибровка)
#define VFO_CALIBRATED_XOSC_HZ 12000350ULL

// Структура параметров частоты для низкоуровневых регистров PIO и таймера фазы
struct VfoParameters {
    uint32_t pio_int;
    uint32_t pio_frac;
    uint32_t dds_step;
};

// Экспорт глобального массива предрассчитанных тонов для прямого доступа из модуляторов
extern VfoParameters ifkp_tones[VFO_IFKP_TONES_COUNT];

// === НАБОР ФУНКЦИЙ УПРАВЛЕНИЯ ВЧ-ЭФИРОМ (Low-Level API) ===

/**
 * Инициализирует аппаратную периферию RP2040: фиксирует PLL процессора на 120 МГц,
 * настраивает конечный автомат PIO на GPIO 28 и запускает фоновый таймер дизеринга фазы.
 */
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz);

/**
 * Мгновенное переключение несущей ВЧ на предрассчитанный тон из таблицы без срыва фазы.
 * ВНИМАНИЕ: Теперь включает внутреннюю проверку на дублирование тона!
 * @param tone_index Индекс тона в массиве (от 0 до 32).
 */
void vfo_set_tone_instant(uint8_t tone_index) __not_in_flash_func();

/**
 * Мгновенная коммутация (замыкание/размыкание) телеграфного ключа для CW.
 */
void vfo_set_cw_key(bool key_down) __not_in_flash_func();

#endif // VFO_HARDWARE_H
