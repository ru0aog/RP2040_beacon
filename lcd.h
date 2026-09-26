/**  
 * ============================================================================  
 *  LCD.h — Публичный интерфейс драйвера символьного дисплея HD44780/PCF8574  
 * ============================================================================  
 *  
 *  Объявляет функции вывода на ЖК-дисплей 1602/2004 через I2C-расширитель  
 *  PCF8574 (адрес LCD_ADDRESS = 0x27). Пины SDA/SCL и флаг наличия дисплея  
 *  берутся из массива device_DL[5] (заполняется I2C-сканером в основном скетче).  
 *  Реализация софтового I2C и протокола HD44780 — в LCD.cpp.  
 *  
 *    LCD_init(deep_init)        — инициализация дисплея.  
 *    LCD_print(text, row, col)  — вывод строки (перегрузки для char* и String).  
 *    LCD_push_char(...)         — вывод символа/строки в режиме бегущей строки.  
 *    lcd_command / lcd_char / lcd_write_nibble — низкоуровневые примитивы.  
 * ============================================================================  
 */

#ifndef LCD_H
#define LCD_H

#include <Arduino.h>

// Адрес датчика на шине I2C.
#define LCD_ADDRESS 0x27

extern uint8_t device_DL[5];

void LCD_init(bool deep_init);
void LCD_print(const char* text, uint8_t row, uint8_t col);
void LCD_print(String text, uint8_t row, uint8_t col); // Добавленная перегрузка для String
void lcd_write_nibble(byte nibble, byte mode);
void lcd_command(byte cmd);
void lcd_char(byte data);
uint8_t LCD_detect_rows();

void LCD_push_char(char new_char, uint8_t row, uint8_t width);
void LCD_push_char(const char* str, uint8_t row, uint8_t width); // Добавленная перегрузка для String

#endif
