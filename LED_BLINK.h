/**  
 * ============================================================================  
 *  LED_BLINK.h — Публичный API светодиодной индикации маяка  
 * ============================================================================  
 *  
 *  Объявляет функции управления индикацией, единые для трёх плат  
 *  (RP2040-Zero / YD-RP2040 — адресный RGB WS2812B; обычная Pico — LED_BUILTIN).  
 *  Реализация и низкоуровневые детали (bit-banging WS2812B, битовые маски,  
 *  порядок цвета G-R-B) — в LED_BLINK.cpp.  
 *  
 *    ZERO_LED_init()      — инициализация GPIO индикаторов.  
 *    ZERO_LED_RED_ON()    — включить красный.  
 *    ZERO_LED_GREEN_ON()  — включить зелёный.  
 *    ZERO_LED_BLUE_ON()   — включить синий.  
 *    ZERO_LED_OFF()       — погасить все индикаторы.  
 * ============================================================================  
 */

#ifndef LED_BLINK_H
#define LED_BLINK_H

#include <Arduino.h>

void ZERO_LED_init();
void ZERO_LED_RED_ON();
void ZERO_LED_GREEN_ON();
void ZERO_LED_BLUE_ON();
void ZERO_LED_OFF();

#endif
