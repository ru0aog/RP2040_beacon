/**  
 * ============================================================================  
 *  scheduler.h — Публичный интерфейс RTC и планировщика передач маяка  
 * ============================================================================  
 *  
 *  Объявляет API работы со временем и расписанием; реализация — в scheduler.cpp.  
 *  Поддерживает внешние чипы DS3231/DS1307A по I2C и внутренний RTC RP2040.  
 *  
 *  КОНСТАНТЫ:  
 *    RTC_I2C_ADDRESS (0x68) — адрес чипа часов на шине I2C.  
 *  
 *  ГЛОБАЛЬНОЕ СОСТОЯНИЕ (extern, заполняется update_scheduler):  
 *    rtc_hour/min/sec, rtc_year/month/day — текущее время и дата.  
 *    device_DS[5] — дескриптор часов: [0]=найден, [1]=номер шины (Wire/Wire1),  
 *                   [2]=SDA, [3]=SCL, [4]=адрес.  
 *    rtc_chip_name — имя чипа (из основного INO-скетча).  
 *  
 *  API:  
 *    init_scheduler()          — обнаружение чипа и запуск источника времени.  
 *    update_scheduler()        — чтение времени в глобальные переменные.  
 *    print_current_time/date() — вывод в Serial.  
 *    get_current_time/date()   — строки времени/даты.  
 *    get_telemetry_string()    — строка телеметрии (темп. DS3231 + CPU).  
 *    is_time_to_transmit(mode) — проверка расписания (0=IFKP, 1=RTTY, 2=CW).  
 *    handle_time_command / handle_date_command — установка времени/даты из UART.  
 *    I2C_DS_restart()          — переинициализация шины Wire/Wire1 часов.  
 * ============================================================================  
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>
#include <Wire.h>

// Адрес часов на шине I2C
#define RTC_I2C_ADDRESS 0x68

// Прототипы функций планировщика маяка
void init_scheduler();
void update_scheduler();
void print_current_time();
void print_current_date();
String get_telemetry_string();
String get_current_time();
String get_current_date();

extern uint8_t  rtc_hour;
extern uint8_t  rtc_min;
extern uint8_t  rtc_sec;
extern uint16_t rtc_year;
extern uint8_t  rtc_month;
extern uint8_t  rtc_day;
extern uint8_t  rtc_dotw; // Изолированная переменная дня недели (1=Пн ... 7=Вс)

extern uint8_t device_DS[5];
// Тип активного источника времени (реализация в scheduler.cpp)  
enum RtcType {  
    RTC_NONE, 
    RTC_INTERNAL, 
    RTC_DS1307, 
    RTC_DS3231
};  
  
extern RtcType activeRtc;

extern String rtc_chip_name; // будем пользоваться переменной из основного INO-файла

void I2C_DS_restart(); // переключить шину Wire1 на устройство DS3231
void handle_time_command(String cmd);
void handle_date_command(String cmd);
bool is_time_to_transmit(uint8_t mode);


// scheduler.h — добавить  
#define MAX_SCHEDULE_TASKS 32  
enum BeaconMode : uint8_t { MODE_IFKP=0, MODE_RTTY=1, MODE_CW=2, MODE_SEQ=3, MODE_NONE=255 };  
  
struct TaskItem {  
  bool     active;  
  uint8_t  mode;          // BeaconMode  
  uint8_t  days;          // битовая маска, бит d = день недели (1..7); 0 = каждый день  
  uint8_t  start_hour, start_min;  
  uint8_t  end_hour,   end_min;  
  uint16_t interval_min;  // 0 = разовая задача  
  uint32_t freq_hz;  
};  
extern TaskItem beacon_schedule[MAX_SCHEDULE_TASKS];

void print_schedule();
static String fmt_next_start(int32_t abs_min);
int32_t get_next_start_minute(uint8_t mode);

/*
// ============================================================================
// Структура планировщика задач нового поколения в ОЗУ
// ============================================================================
#define MODE_CW    0
#define MODE_RTTY  1
#define MODE_IFKP  2
#define MODE_SEQ   3 // Комбинированная цепочка: CW -> 1м -> RTTY -> 1м -> IFKP

// Структура задачи с использованием битовой маски под дни недели (1 байт)
struct TaskItem {
    uint8_t  days;         // Битовая маска дней: 0=Каждый день, бит 1=Пн, бит 2=Вт ... бит 7=Вс
    uint8_t  start_hour;   
    uint8_t  start_min;    
    uint8_t  end_hour;     
    uint8_t  end_min;      
    uint8_t  interval_min; 
    uint32_t freq_hz;      
    uint8_t  mode;         
    bool     active;       
};


#define MAX_SCHEDULE_TASKS 32
extern TaskItem beacon_schedule[MAX_SCHEDULE_TASKS];
*/

#endif
