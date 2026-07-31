#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <Arduino.h>

// Глобальные переменные будут доступны во всех вкладках
extern String my_call_variable;
extern String my_qth_variable;
extern String my_text_variable;
extern String my_rtty_variable;
extern String my_ifkp_variable;
extern String my_freq_ifkp_var;
extern String my_rtty_space_var;
extern String my_rtty_mark_var;
extern String my_cw_variable;      // Строка минут запуска (например, "15:22,17:22")
extern String my_freq_cw_var;      // Строка частоты несущей (например, "3601000")
extern String my_rtty_baud_var;

extern volatile bool pc_file_written; 
extern volatile uint32_t RTTY_BIT_TIME_US;

// Прототипы функций управления файлами
void init_file_manager();
void check_and_handle_pc_changes();
void print_current_settings();
void read_file_to_variable();

#endif
