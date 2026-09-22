#include <hardware/gpio.h>
#include "LED_BLINK.h"

// код передачи
void __time_critical_func(send_raw_byte)(uint8_t byte_val) {
    for (int i = 7; i >= 0; i--) {
        if ((byte_val >> i) & 1) {
            sio_hw->gpio_set = (1 << ZERO_LED_PIN);
            for (volatile int d = 0; d < 22; d++); 
            sio_hw->gpio_clr = (1 << ZERO_LED_PIN);
            for (volatile int d = 0; d < 8; d++);  
        } else {
            sio_hw->gpio_set = (1 << ZERO_LED_PIN);
            for (volatile int d = 0; d < 2; d++); 
            sio_hw->gpio_clr = (1 << ZERO_LED_PIN);
            for (volatile int d = 0; d < 45; d++); 
        }
    }
}

// функция установки цвета
void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    noInterrupts();
    send_raw_byte(g);
    send_raw_byte(r);
    send_raw_byte(b);
    interrupts();
    
    sio_hw->gpio_clr = (1 << ZERO_LED_PIN);
    delayMicroseconds(600); 
}


void ZERO_LED_init() {
// инициализация многоцветного LED
  gpio_init(ZERO_LED_PIN);
  gpio_set_dir(ZERO_LED_PIN, GPIO_OUT);
  
  sio_hw->gpio_clr = (1 << ZERO_LED_PIN);
  delay(200); 
}

void ZERO_LED_RED_ON() {
  set_rgb(255, 0, 0);
}

void ZERO_LED_GREEN_ON() {
  set_rgb(0, 255, 0);
}

void ZERO_LED_OFF() {
  set_rgb(0, 0, 0);
}

/*

  const int steps = 50;        // 50 шагов для идеальной плавности
  const int step_delay = 10;   // 10 мс на шаг (50 * 10 мс = 500 мс)
  const int hold_delay = 300;  // Время фиксации на пике (300 мс)
  const int max_val = 120;     // Максимальная сочная яркость (из 255)

  // ==========================================
  // КРАСНЫЙ ЦВЕТ
  // ==========================================
  // 1. Нарастание (500 мс)
  for (int i = 0; i <= steps; i++) {
    uint8_t val = (i * i * max_val) / (steps * steps);
    set_rgb(val, 0, 0);
    delay(step_delay);
  }
  // 2. Фиксация
  delay(hold_delay);
  // 3. Спад (500 мс)
  for (int i = steps; i >= 0; i--) {
    uint8_t val = (i * i * max_val) / (steps * steps);
    set_rgb(val, 0, 0);
    delay(step_delay);
  }
  delay(100); // Короткая пауза в темноте перед сменой цвета

  // ==========================================
  // ЗЕЛЕНЫЙ ЦВЕТ
  // ==========================================
  // 1. Нарастание (500 мс)
  for (int i = 0; i <= steps; i++) {
    uint8_t val = (i * i * max_val) / (steps * steps);
    set_rgb(0, val, 0);
    delay(step_delay);
  }
  // 2. Фиксация
  delay(hold_delay);
  // 3. Спад (500 мс)
  for (int i = steps; i >= 0; i--) {
    uint8_t val = (i * i * max_val) / (steps * steps);
    set_rgb(0, val, 0);
    delay(step_delay);
  }
  delay(100);

  // ==========================================
  // СИНИЙ ЦВЕТ
  // ==========================================
  // 1. Нарастание (500 мс)
  for (int i = 0; i <= steps; i++) {
    uint8_t val = (i * i * max_val) / (steps * steps);
    set_rgb(0, 0, val);
    delay(step_delay);
  }
  // 2. Фиксация
  delay(hold_delay);
  // 3. Спад (500 мс)
  for (int i = steps; i >= 0; i--) {
    uint8_t val = (i * i * max_val) / (steps * steps);
    set_rgb(0, 0, val);
    delay(step_delay);
  }
  delay(400); // Пауза перед началом нового круга в loop

  Serial.println("Последовательный порт активен");
*/
