#include <hardware/gpio.h>
#include "LED_BLINK.h"

// Определяем маски пинов для всех трех типов плат одновременно
// GPIO16 (Zero), GPIO23 (YD-RP2040), GPIO25 (Обычная Pico / LED_BUILTIN)
#define PIN_ZERO      16
#define PIN_YD        23
#define PIN_PICO      25

// Битовая маска для одновременного управления адресными светодиодами (16 и 23)
const uint32_t RGB_PINS_MASK = (1 << PIN_ZERO) | (1 << PIN_YD);

// Битовая маска вообще для всех используемых светодиодов (16, 23 и 25)
const uint32_t ALL_PINS_MASK = (1 << PIN_ZERO) | (1 << PIN_YD) | (1 << PIN_PICO);

// Параллельная высокоскоростная отправка байта на пины 16 и 23 одновременно
void __time_critical_func(send_raw_byte_parallel)(uint8_t byte_val) {
    for (int i = 7; i >= 0; i--) {
        if ((byte_val >> i) & 1) {
            // Передача логической '1'
            sio_hw->gpio_set = RGB_PINS_MASK;       // Выставляем 16 и 23 в HIGH
            for (volatile int d = 0; d < 22; d++); 
            sio_hw->gpio_clr = RGB_PINS_MASK;       // Сбрасываем 16 и 23 в LOW
            for (volatile int d = 0; d < 8; d++);  
        } else {
            // Передача логического '0'
            sio_hw->gpio_set = RGB_PINS_MASK;       // Выставляем 16 и 23 в HIGH
            for (volatile int d = 0; d < 2; d++); 
            sio_hw->gpio_clr = RGB_PINS_MASK;       // Сбрасываем 16 и 23 в LOW
            for (volatile int d = 0; d < 45; d++); 
        }
    }
}

// Универсальная функция установки состояния индикаторов
void set_rgb_parallel(uint8_t r, uint8_t g, uint8_t b) {
    // 1. Отправляем битовую посылку на адресные светодиоды
    noInterrupts();
    send_raw_byte_parallel(g); // WS2812B принимает первым байт зеленого (G)
    send_raw_byte_parallel(r); // Затем красного (R)
    send_raw_byte_parallel(b); // Затем синего (B)
    interrupts();
    
    // Формируем паузу (Latch) для фиксации цвета в чипах WS2812B
    sio_hw->gpio_clr = RGB_PINS_MASK;
    delayMicroseconds(600); 

    // 2. Управляем обычным монохромным светодиодом на GPIO25
    // Если запрашивается любой цвет, отличный от нуля — зажигаем его
    if (r > 0 || g > 0 || b > 0) {
        sio_hw->gpio_set = (1 << PIN_PICO);
    } else {
        sio_hw->gpio_clr = (1 << PIN_PICO);
    }
}

// Аппаратная инициализация всех трех каналов индикации
void ZERO_LED_init() {
  Serial.println("[Система] Индикация светодиодом: универсальный режим (YD-RP2040, Pico, Zero)");

  // Инициализируем GPIO16
  gpio_init(PIN_ZERO);
  gpio_set_dir(PIN_ZERO, GPIO_OUT);

  // Инициализируем GPIO23
  gpio_init(PIN_YD);
  gpio_set_dir(PIN_YD, GPIO_OUT);

  // Инициализируем GPIO25
  gpio_init(PIN_PICO);
  gpio_set_dir(PIN_PICO, GPIO_OUT);
  
  // Переводим все выводы в состояние LOW
  sio_hw->gpio_clr = ALL_PINS_MASK;
  
  delay(200); 
}

void ZERO_LED_RED_ON() {
  // На Zero и YD-RP2040 загорится красный, на обычной Pico — включится встроенный светодиод
  set_rgb_parallel(255, 0, 0);
}

void ZERO_LED_GREEN_ON() {
  // На Zero и YD-RP2040 загорится зеленый, на обычной Pico — включится встроенный светодиод
  set_rgb_parallel(0, 255, 0);
}

void ZERO_LED_BLUE_ON() {
  // На Zero и YD-RP2040 загорится синий, на обычной Pico — включится встроенный светодиод
  set_rgb_parallel(0, 0, 255);
}

void ZERO_LED_OFF() {
  // Гасим абсолютно все индикаторы на плате
  set_rgb_parallel(0, 0, 0);
}
