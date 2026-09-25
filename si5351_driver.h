/**  
 * ============================================================================  
 *  si5351_driver.h — Публичный интерфейс драйвера синтезатора Si5351  
 * ============================================================================  
 *  
 *  Объявляет Low-Level API управления Si5351 по I2C; реализация — в  
 *  si5351_driver.cpp. При отсутствии чипа управление ВЧ-выходом уходит на  
 *  программный VFO RP2040 (vfo_hardware).  
 *  
 *  КОНСТАНТЫ:  
 *    SI5351_I2C_ADDR (0x60) — адрес синтезатора на шине I2C.  
 *  
 *  ГЛОБАЛЬНОЕ СОСТОЯНИЕ (extern):  
 *    SI_FAIL       — флаг «чип не найден/не инициализирован».  
 *    device_SI[5]  — дескриптор генератора: [0]=найден, [1]=номер шины  
 *                    (Wire/Wire1), [2]=SDA, [3]=SCL, [4]=адрес.  
 *  
 *  API:  
 *    init_si5351()               — обнаружение и полная настройка чипа.  
 *    I2C_SI_restart()            — переинициализация шины Wire/Wire1.  
 *    si5351_write_reg(reg, data) — запись регистра с повторами при сбое I2C.  
 *    setFrq_si5351(SI_FREQ[], CLK_NO) — быстрая отправка 8-байтного пакета  
 *                                       частоты на CLK0/CLK1.  
 *    CLK_ON_si5351 / CLK_OFF_si5351(CLK_NO) — вкл/выкл выхода (или VFO-ключ).  
 *    SI_POWER_ON / SI_POWER_OFF() — подача/снятие питания с выходов.  
 *    make_freq_with_space(FREQ)  — строковое форматирование частоты.  
 * ============================================================================  
 */

#ifndef SI5351_DRIVER_H
#define SI5351_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

#define SI5351_I2C_ADDR    0x60

extern bool SI_FAIL;
extern uint8_t device_SI[5];

// Прототипы функций модуля
void I2C_SI_restart(); // переключить шину Wire1 на устройство Si5351
void CLK_OFF_si5351(uint8_t CLK_NO); // выключить выход CLK_NO
void CLK_ON_si5351(uint8_t CLK_NO);  // включить  выход CLK_NO
void init_si5351();
void setFrq_si5351(uint8_t SI_FREQ[], uint8_t CLK_NO);
void SI_POWER_ON();
void SI_POWER_OFF();
bool si5351_write_reg(uint8_t reg, uint8_t data);
String make_freq_with_space(uint32_t FREQ);

#endif
