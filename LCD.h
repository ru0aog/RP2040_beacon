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