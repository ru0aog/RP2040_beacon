/**  
 * ============================================================================  
 *  file_manager.h — Интерфейс USB-MSC хранилища конфигурации маяка  
 * ============================================================================  
 *  
 *  Объявляет глобальное состояние и API модуля; реализация — в file_manager.cpp.  
 *  
 *  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (заполняются из INFO.TXT, читаются всеми модулями):  
 *    my_call_variable / my_qth_variable / my_text_variable — позывной, локатор, текст.  
 *    my_cw_variable / my_rtty_variable / my_ifkp_variable  — расписания запуска  
 *        (строки минут, напр. "15:22,17:22").  
 *    my_freq_cw_var / my_freq_ifkp_var                     — частоты несущих.  
 *    my_rtty_space_var / my_rtty_mark_var                  — частоты SPACE/MARK.  
 *    my_rtty_baud_var                                      — скорость RTTY (Бод).  
 *    my_FAT                                                — статус файловой системы.  
 *    pc_file_written    — флаг активности ПК (аварийный останов передачи).  
 *    RTTY_BIT_TIME_US   — длительность бита RTTY, рассчитана из скорости.  
 *  
 *  API:  
 *    init_file_manager()          — загрузка образа из Flash (или генерация  
 *                                   диска по умолчанию), старт USB-MSC, разбор конфига.  
 *    check_and_handle_pc_changes()— обработка правок с ПК в loop() (сохранение  
 *                                   во Flash + перечитывание + передёргивание тома).  
 *    print_current_settings()     — вывод текущих настроек в Serial.  
 *    read_file_to_variable()      — разбор INFO.TXT в глобальные переменные.  
 * ============================================================================  
 */

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
extern String my_rtty_mark_var;    // Базовая частота MARK
extern String my_rtty_shift_var;   // Расстояние между частотами (Гц)
extern String my_rtty_invert_var;  // Флаг инверсии частот RTTY (0 или 1)
extern String my_cw_variable;      // Строка минут запуска (например, "15:22,17:22")
extern String my_freq_cw_var;      // Строка частоты несущей (например, "3601000")
extern String my_rtty_baud_var;
extern String my_FAT;

extern volatile bool pc_file_written; 
extern volatile uint32_t RTTY_BIT_TIME_US;

// Прототипы функций управления файлами
void init_file_manager();
void check_and_handle_pc_changes();
void print_current_settings();
void read_file_to_variable();

// API для работы с журналом LOG.TXT
void log_file_clear();
void log_file_write_line(String message);
extern bool is_transmitting; // Ссылка на флаг занятости эфира из главного скетча

// ГЕОМЕТРИЯ И ЛИМИТЫ ФАЙЛОВОЙ СИСТЕМЫ МАЯКА
#define INFO_FIRST_CLUSTER   2
#define INFO_CLUSTERS        20    // 20 секторов = 10 КБ под конфигурацию
#define INFO_MAX_BYTES       (INFO_CLUSTERS * 512)

#define LOG_FIRST_CLUSTER    22    // Идет сразу за INFO (2 + 20)
#define LOG_CLUSTERS         100   // 100 секторов = 50 КБ под логи работы
#define LOG_MAX_BYTES        (LOG_CLUSTERS * 512)

// ============================================================================
// Константы и метаданные для циклического выравнивания износа (Wear Leveling)
// ============================================================================
#define FLASH_SLOTS         8
#define SLOT_SIZE           DISK_SIZE_BYTES // 128 КБ (кратно размеру стирания 4 КБ)
#define SLOT_MAGIC          0xBEA1C011      // Уникальный маркер валидности слота

struct __attribute__((packed)) SlotMeta {
  uint32_t magic;
  uint32_t seq;
};

extern int32_t current_active_slot;
extern uint32_t current_max_seq;


#endif
