/**
 * @file jtty_modem.h
 * @brief JTTY Async 4-GFSK modem implementation for RP2040 beacon
 * @author Your Name / @ru0aog
 * @date 2026-10-15
 * @license GPLv3
 * 
 * Target: Raspberry Pi Pico / RP2040-Zero
 * Project: https://github.com/WSJTX/wsjtx/tree/release/3.3
 */

#ifndef JTTY_MODEM_H
#define JTTY_MODEM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация и запуск циклической передачи данных в моде JTTY.
 * 
 * @param freq_hz Базовая несущая частота передачи (в Гц).
 * @param text Указатель на текстовую строку для отправки в эфир.
 */
void jtty_modem_start(uint32_t freq_hz, const char* text);

/**
 * @brief Функция циклической обработки (тайминги символов и GFSK интерполяция).
 * Должна непрерывно вызываться внутри основного loop() прошивки.
 */
void jtty_modem_process(void);

/**
 * @brief Принудительная или плановая остановка передачи JTTY.
 * Выключает VFO и переводит логику маяка в режим ожидания.
 */
void jtty_modem_stop(void);

/**
 * @brief Проверка текущего статуса работы модема JTTY.
 * 
 * @return true Если в данный момент идет передача кадра в эфир.
 * @return false Если модем находится в режиме ожидания.
 */
bool jtty_modem_is_active(void);

#ifdef __cplusplus
}
#endif

#endif // JTTY_MODEM_H
