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
  
extern bool debug_flag;   // [DEBUG]=1 в SET.TXT: вывод служебных сообщений

extern volatile bool pc_file_written; 
extern volatile uint32_t RTTY_BIT_TIME_US;

// Прототипы функций управления файлами
void init_usb_msc_interface();      // инициализация USB-регистрацию диска
void init_flash_disk();             // Считывание данных в ОЗУ из Flash
void init_file_manager();           // инициализация файл-менеджера
void force_reset_to_default_disk(); // Принудительный сброс USB-диска на дефолт
void check_and_handle_pc_changes();
void print_current_settings();
void read_and_parse_INFO_txt();   // парсер INFO.txt
void read_and_parse_SET_txt();    // парсер SET.txt
void remount_usb_disk ();         // перемонтировать USB-диск
void ensure_file_timestamps();    // проверяет метки даты/времени на файлах


// API для работы с журналом LOG.TXT
void log_file_clear();
void log_file_write_line(String message);
extern bool is_transmitting; // Ссылка на флаг занятости эфира из главного скетча

// ============================================================================
// Геометрия диска 128 КБ (Каталог = 48 записей, Данные = Сектор 5)
// ============================================================================
#define INFO_FIRST_CLUSTER   2
#define INFO_CLUSTERS        20    // 20 секторов = 10 КБ под конфигурацию
#define INFO_MAX_BYTES       (INFO_CLUSTERS * 512)

#define SET_FIRST_CLUSTER    22    // Идет сразу за INFO (2 + 20)
#define SET_CLUSTERS         20    // 20 секторов = 10 КБ под инженерные настройки
#define SET_MAX_BYTES        (SET_CLUSTERS * 512)

#define LOG_FIRST_CLUSTER    42    // Идет сразу за SET (22 + 20)
#define LOG_CLUSTERS         200   // 200 секторов = 100 КБ под логи работы
#define LOG_MAX_BYTES        (LOG_CLUSTERS * 512)

// Определение параметров геометрии диска в ОЗУ
#define SECTOR_SIZE        512
#define SECTOR_COUNT       256   // 128 КБ
#define DISK_SIZE_BYTES    (SECTOR_COUNT * SECTOR_SIZE)
#define FLASH_TARGET_OFFSET (FS_START - 0x10000000)


// ============================================================================
// Служебная зона FLASH-диска
// ============================================================================
struct __attribute__((packed)) SlotMeta {  
  uint32_t magic;  
  uint32_t seq;  
  uint32_t max_clk_sys_hz;   // доказанный OCTEST потолок шины (−3% от достигнутого)  
  uint32_t max_pll_vco_hz;   // доказанный PLLTEST потолок VCO (−3% от достигнутого)  
}; 

// Константы и метаданные для циклического выравнивания износа (Wear Leveling)
#define FLASH_SLOTS         8
#define SLOT_SIZE           DISK_SIZE_BYTES // 128 КБ (кратно размеру стирания 4 КБ)
#define SLOT_MAGIC          0xBEA1C011      // Уникальный маркер валидности слота

#define PASSPORT_CLK_SYS_MAX_HZ  133000000UL   // паспортный потолок clk_sys  
#define PASSPORT_PLL_VCO_MAX_HZ 1600000000UL   // паспортный потолок VCO 

// Действующие лимиты разгона: загружаются из Flash, до прогона тестов — паспортные  
extern uint32_t vfo_max_clk_sys_hz;  
extern uint32_t vfo_max_pll_vco_hz;  
  
// Записать достигнутые лимиты во Flash (вызывается из шагов OCTEST/PLLTEST)  
void persist_clock_limits(uint32_t clk_sys_hz, uint32_t pll_vco_hz);

// доступ к переменным из других модулей
extern int32_t current_active_slot;
extern uint32_t current_max_seq;
extern String BCN_VER;
extern String BCN_DAT;


// ============================================================================
// Глобальные переменные инженерных пинов и результатов сканирования
// ============================================================================
extern int pin_freq_out;
extern int pin_amp_act;
extern int subband_pins[4];
extern int pin_pwr_si;
extern int pin_pwr_ds;
extern int pin_pwr_bm;
extern int pin_pwr_dl;
extern String scan_exclude_list;
extern String scan_result_data;
// НОВЫЕ ДИНАМИЧЕСКИЕ ИНТЕРВАЛЫ КЛИМАТА
extern uint32_t climate_sample_interval_min; // Интервал снятия логов (минуты)
extern uint32_t climate_report_interval_min; // Интервал сброса таблицы в LOG.TXT (минуты)

// Прототип новой функции парсинга
void read_hardware_settings();

// Функция экспорта результатов сканирования I2C и пинов обратно в SET.TXT
void save_hardware_settings_to_file(String scan_results);

void log_file_write_block(const String& block);  // запись многострочного блока на флэш за один раз

bool is_pin_excluded_from_scan(int pin);


extern bool led_enable_flag;    // Флаг включения светодиодной индикации (1 — вкл, 0 — выкл)
extern int  si5351_clk_tx_out;  // Номер активного выхода Si5351 на TX (0 = CLK0, 1 = CLK1, 2 = CLK2)



#endif
