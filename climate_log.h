#ifndef CLIMATE_LOG_H
#define CLIMATE_LOG_H

#include <Arduino.h>

void climate_log_update(); // прототип

void climate_print_table_to_console();
void climate_send_table_to_air();
String climate_build_current_table();

#endif