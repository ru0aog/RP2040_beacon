#include "LCD.h"

#define PIN_RS     0x01  // Бит выбора регистра (0 - команда, 1 - данные)
#define PIN_RW     0x02  // Бит Чтения/Записи
#define PIN_EN     0x04  // Бит стробирования (Enable)
#define BACKLIGHT  0x08  // Бит управления подсветкой дисплея (0x08 - ВКЛ, 0x00 - ВЫКЛ)

// драйвер вывода текста на LCD-дисплей через I2C-контроллер шины 8574
// так как питание модуля +5В, то подтяжка линий к +5В с модулей снята!
// переключение состояний линий выполняется атомарно



// Достаточная задержка, чтобы слабая внутренняя подтяжка RP2040 
// успевала полностью зарядить емкость шины без внешних резисторов
void i2c_delay() {
  delayMicroseconds(10); 
}

// Безопасный перевод линии в состояние 1 (HIGH) через INPUT_PULLUP
void i2c_high(int pin) {
  pinMode(pin, INPUT_PULLUP);
}

// Безопасный перевод линии в состояние 0 (LOW) через OUTPUT без коротких замыканий
void i2c_low(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

// Программный старт шины I2C на динамических пинах из массива
void soft_i2c_start() {
  i2c_high(device_DL[2]); i2c_delay(); // SDA
  i2c_high(device_DL[3]); i2c_delay(); // SCL
  i2c_low(device_DL[2]);  i2c_delay(); // Падение SDA при высоком SCL
  i2c_low(device_DL[3]);  i2c_delay(); // Падение SCL
}

// Программный стоп шины I2C
void soft_i2c_stop() {
  i2c_low(device_DL[2]);  i2c_delay(); // SDA
  i2c_high(device_DL[3]); i2c_delay(); // SCL
  i2c_high(device_DL[2]); i2c_delay(); // Подъем SDA при высоком SCL
}

// Программная отправка одного байта в режиме Open-Drain
void soft_i2c_write(byte data) {
  for (int i = 0; i < 8; i++) {
    if (data & 0x80) {
      i2c_high(device_DL[2]);
    } else {
      i2c_low(device_DL[2]);
    }
    i2c_delay();
    i2c_high(device_DL[3]); i2c_delay();
    i2c_low(device_DL[3]);  i2c_delay();
    data <<= 1;
  }
  
  // 9-й такт (ACK) — полностью отпускаем линию в режим входа с подтяжкой.
  // Это гарантирует защиту процессора от зависания при встречном ответе от PCF8574.
  i2c_high(device_DL[2]); 
  i2c_delay();
  i2c_high(device_DL[3]); i2c_delay();
  i2c_low(device_DL[3]);  i2c_delay();
}

// Низкоуровневая отправка полубайта в PCF8574 (разбита на два независимых пакета старт/стоп)
void lcd_write_nibble(byte nibble, byte mode) {
  byte data = (nibble & 0xF0) | mode | BACKLIGHT; // 0x08 - подсветка ВКЛ

  // Пакет 1: Передаем данные и поднимаем строб EN в 1
  soft_i2c_start();
  soft_i2c_write(LCD_ADDRESS << 1); 
  soft_i2c_write(data | PIN_EN); 
  soft_i2c_stop();
  delay(1); // Даем время зафиксировать высокий уровень

  // Пакет 2: Опускаем строб EN в 0 (фиксация контроллером HD44780 по спаду)
  soft_i2c_start();
  soft_i2c_write(LCD_ADDRESS << 1);
  soft_i2c_write(data & ~PIN_EN); 
  soft_i2c_stop();
  delayMicroseconds(10); // Время на выполнение внутренней микрокоманды дисплея
}

// Отправка полного байта (разбиваем на два полубайта)
void lcd_send(byte value, byte mode) {
  lcd_write_nibble(value & 0xF0, mode);        // Старший полубайт
  lcd_write_nibble((value << 4) & 0xF0, mode); // Младший полубайт
}

void lcd_command(byte cmd) { lcd_send(cmd, 0); }
void lcd_char(byte data)   { lcd_send(data, PIN_RS); }

// Функция установки курсора в нужную позицию (столбец, строка)
void lcd_set_cursor(byte col, byte row) {
  // Расширенный массив смещений для 4-строчного дисплея (2004)
  // Строка 0: 0x00, Строка 1: 0x40, Строка 2: 0x14, Строка 3: 0x54
  byte row_offsets[] = {0x00, 0x40, 0x14, 0x54}; 
  // Защита от выхода за границы массива (вдруг передадут row = 4 или больше)
  if (row >= 4) {
    row = 3; 
  }
  // Отправляем команду установки адреса DDRAM (0x80)
  lcd_command(0x80 | (col + row_offsets[row]));
}


// Вывод обычной строки текста
void lcd_print(const char* str) {
  while (*str) {
    lcd_char(*str++);
  }
}


// Программное чтение одного байта по шине I2C
uint8_t soft_i2c_read() {
  uint8_t data = 0;

  // Отпускаем линию SDA (переводим в high / вход с подтяжкой), чтобы ведомый мог ей управлять
  i2c_high(device_DL[2]); 
  i2c_delay();

  for (int i = 0; i < 8; i++) {
    i2c_high(device_DL[3]); // Поднимаем SCL (микросхема выставила бит)
    i2c_delay();

    data <<= 1;
    // Считываем физическое состояние пина SDA с помощью digitalRead
    if (digitalRead(device_DL[2]) == HIGH) { 
      data |= 0x01;
    }

    i2c_low(device_DL[3]);  // Опускаем SCL
    i2c_delay();
  }

  // 9-й такт (ACK/NACK от мастера)
  // При чтении последней тетрады принято отправлять NACK (отпускаем SDA в high),
  // чтобы сказать микросхеме, что чтение окончено.
  i2c_high(device_DL[2]); i2c_delay();
  i2c_high(device_DL[3]); i2c_delay();
  i2c_low(device_DL[3]);  i2c_delay();

  return data;
}



// Функция чтения одной 4-битной тетрады (nibble) с дисплея
uint8_t lcd_read_nibble(uint8_t mode) {
  uint8_t nibble = 0;
  
  // --- Шаг 1. Переводим PCF8574 в режим чтения (выставляем нужные пины управления) ---
  // Нам нужно поднять PIN_RW и строб PIN_EN в состояние HIGH
  soft_i2c_start(); // Функция генерации СТАРТ-условия (должна быть у вас в коде)
  soft_i2c_write(LCD_ADDRESS << 1); // Адрес на запись (например, 0x27 << 1 = 0x4E)
  soft_i2c_write(mode | PIN_RW | PIN_EN | BACKLIGHT); 
  soft_i2c_stop();  // Функция генерации СТОП-условия
  
  delayMicroseconds(5); // Небольшая пауза для стабилизации логических уровней
  
  // --- Шаг 2. Читаем данные, которые PCF8574 выставила на своих пинах ---
  soft_i2c_start();
  soft_i2c_write((LCD_ADDRESS << 1) | 0x01); // Переключаем шину в режим ЧТЕНИЯ (младший бит = 1)
  uint8_t pcf_data = soft_i2c_read();
  soft_i2c_stop();
  
  // Старшие 4 бита расширителя (P4-P7) содержат полубайт от LCD
  nibble = pcf_data & 0xF0; 
  
  // --- Шаг 3. Завершаем операцию чтения (сбрасываем строб PIN_EN в LOW) ---
  soft_i2c_start();
  soft_i2c_write(LCD_ADDRESS << 1);
  soft_i2c_write(mode | PIN_RW | BACKLIGHT);
  soft_i2c_stop();
  
  delayMicroseconds(5);
  
  return nibble;
}



// Функция чтения полноценного байта (команды/статуса) из дисплея
uint8_t lcd_read_status() {
  // Читаем в режиме команд (RS = 0)
  uint8_t high = lcd_read_nibble(0);
  uint8_t low  = lcd_read_nibble(0);
  
  // Собираем байт: старший ниббл на своем месте, младший сдвигаем вправо
  return (high | (low >> 4));
}






void LCD_init(bool deep_init) {
  // Инициализируем дисплей
  if (device_DL[0] == 1) {
    // На старте жестко переводим линии в высокое безопасное состояние
    i2c_high(device_DL[2]);
    i2c_high(device_DL[3]);
    //delay(500); // Ожидание полной стабилизации питания экрана

    // пошаговая инициализация HD44780 в 4-битном режиме
    lcd_write_nibble(0x30, 0); delay(15); // Сброс 1 (нужно > 4.1 мс)
    lcd_write_nibble(0x30, 0); delay(5);  // Сброс 2 (нужно > 100 мкс)
    lcd_write_nibble(0x30, 0); delay(5);  // Сброс 3
    lcd_write_nibble(0x20, 0); delay(5);  // Окончательное включение 4-битного режима

    // Настройка параметров дисплея
    lcd_command(0x28); delay(5);  // 2 строки, матрица 5x8 (работает и на 2004)
    lcd_command(0x0C); delay(5);  // Дисплей включен, курсор выключен
    lcd_command(0x06); delay(5);  // Направление движения курсора — вправо
    if (deep_init) {lcd_command(0x01); delay(20);} // Глубокая очистка памяти DDRAM (нужно > 15 мс)
  }
}


void LCD_print(const char* text, uint8_t row, uint8_t col) {
  // Проверяем, что дисплей подключен и адрес совпадает
  if (device_DL[4] != LCD_ADDRESS) {
    return; // Если дисплея нет, выходим из функции
  }
  // 1. Устанавливаем курсор в заданную позицию
  lcd_set_cursor(col, row); 
  // 2. Печатаем переданный текст
  lcd_print(text);
}

void LCD_print(String text, uint8_t row, uint8_t col) {
  // Просто перевызываем базовую функцию, используя .c_str()
  LCD_print(text.c_str(), row, col);
}



// Глобальный буфер для 4 строк по 20 символов + 1 байт на ноль-терминатор
// 20 символов + 1 байт под нуль-терминатор = 21
char lcd_scroll_buffers[4][21] = {
  "                    ", // 20 пробелов
  "                    ",
  "                    ",
  "                    "
};





// Функция посимвольного сдвига строки влево
// new_char - новый символ, row - номер строки (0-3), width - ширина экрана (16 или 20)
void LCD_push_char(char new_char, uint8_t row, uint8_t width) {
  width = 20; // Жестко фиксируем ширину экрана на 20 символов
  if (row >= 4) return;

  // ИГНОРИРУЕМ управляющие символы (перенос строки \n, \r, табы и т.д.)
  if (new_char < 0x20) {
    return; // Просто выходим, не сдвигая строку
  }

  // Счетчик сдвигов (сохраняет значение между вызовами функции)
  static uint8_t shift_counter = 0;

  // --- ЛОГИКА КАСКАДНОГО СДВИГА (Из Строки 1 в Строку 0) ---

  // 1. Запоминаем самый левый символ нижней строки (индекс 1)
  char overflow_char = lcd_scroll_buffers[1][0];

  // 2. Сдвигаем ВЕРХНЮЮ строку (индекс 0) влево на 1 позицию
  for (int i = 0; i < width - 1; i++) {
    lcd_scroll_buffers[0][i] = lcd_scroll_buffers[0][i + 1];
  }
  // В самый правый край верхней строки переносим символ, ушедший с нижней
  lcd_scroll_buffers[0][width - 1] = overflow_char;

  // 3. Сдвигаем НИЖНЮЮ строку (индекс 1) влево на 1 позицию
  for (int i = 0; i < width - 1; i++) {
    lcd_scroll_buffers[1][i] = lcd_scroll_buffers[1][i + 1];
  }
  // В освободившийся правый край нижней строки записываем новый символ
  lcd_scroll_buffers[1][width - 1] = new_char;

  // --- УВЕЛИЧЕНИЕ СЧЕТЧИКА И ПЕРЕИНИЦИАЛИЗАЦИЯ ---
  //shift_counter++;
  //if (shift_counter >= 10) {
  //  shift_counter = 0; // Сбрасываем счетчик
    LCD_init(false);        // Принудительно "освежаем" контроллер дисплея по I2C
  //}

  // --- МГНОВЕННЫЙ ВЫВОД НА ЭКРАН ---
  LCD_print(lcd_scroll_buffers[0], 0, 0); // Обновляем верхнюю строку
  LCD_print(lcd_scroll_buffers[1], 1, 0); // Обновляем нижнюю строку
}




// Перегруженная функция для автоматического посимвольного вывода строк
void LCD_push_char(const char* str, uint8_t row, uint8_t width) {
  if (str == nullptr) return;
  
  // Цикл идет по строке до тех пор, пока не встретит ноль-терминатор '\0'
  while (*str != '\0') {
    LCD_push_char(*str, row, width); // Вызываем базовую функцию для одиночного char
    str++; // Сдвигаем указатель на следующий символ строки
  }
}




