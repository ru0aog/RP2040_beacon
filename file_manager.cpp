/**  
 * ============================================================================  
 *  file_manager.cpp — Эмуляция USB-MSC (FAT12) и Wear Leveling во Flash  
 *  Версия 2.12 от 2026-10-01, автор RU0AOG  
 * ============================================================================
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Организует виртуальную USB-флешку (128 КБ) в ОЗУ микроконтроллера для ПК,  
 *  обеспечивая прозрачное чтение/запись файлов конфигурации INFO.TXT,  
 *  инженерных параметров SET.TXT и файла журнала LOG.TXT без сторонних библиотек.
 *  
 *  АРХИТЕКТУРА «RAM-ДИСК + FLASH»  
 *  ------------------------------  
 *  Диск целиком живёт в ОЗУ (ram_disk_buffer, 256 секторов × 512 Б = 128 КБ) и  
 *  вручную размечен под FAT12 (Boot Sector + BPB, таблица FAT, корневой каталог,  
 *  область данных — create_default_fat_with_info_file). USB-хост читает/пишет  
 *  секторы через колбэки TinyUSB (msc_read_cb / msc_write_cb / msc_flush_cb).  
 *  Энергонезависимость — вручную: образ ОЗУ сбрасывается во Flash RP2040  
 *  (save_ram_to_flash: flash_range_erase + flash_range_program под запретом  
 *  прерываний), а при старте загружается обратно (init_file_manager).  
 *  Валидность образа проверяется по сигнатуре 0x55/0xAA в конце сектора 0;  
 *  если её нет — генерируется диск по умолчанию с шаблонным INFO.TXT.  
 *  
 *  МЕХАНИЗМ ВЫРАВНИВАНИЯ ИЗНОСА (Wear Leveling)  
 *  --------------------------------------------  
 *  Для предотвращения деградации Flash-памяти RP2040 область файловой системы  
 *  разбита на 8 независимых циклических слотов по 128 КБ. При каждом сохранении  
 *  менеджер ищет следующий слот, проверяет валидность структуры, внедряет в скрытый  
 *  хвост сектора 255 структуру SlotMeta (с Magic-числом и счетчиком версий seq)  
 *  и атомарно перезаписывает сектор Flash. При старте загружается слот с максимальным seq.  
 *  
 *  ПАРСИНГ И ДИНАМИЧЕСКОЕ РЕДАКТИРОВАНИЕ  
 *  --------------------------------------  
 *  - read_and_parse_INFO_txt: Выполняет разбор тегов. Текстовые поля очищаются,  
 *    числовые фильтруются. CSV-парсер разбирает матрицу расписания [TASK_XX]  
 *    (до 32 задач): парсит маски дней (1..7), время (разовое или периодическое  
 *    с интервалами через знак '/'), частоты и типы мод (CW, RTTY, IFKP, SEQ).  
 *  - read_and_parse_SET_txt: Читает конфигурацию пинов (FREQ_OUT, AMP_ACT, subband,  
 *    питание модулей). Поддерживает строковый флаг "NC" (-1) для отключенных цепей.  
 *  - update_info_config_from_console: Ищет маркер в ОЗУ, сдвигает память через memmove  
 *    и перезаписывает размер файла в дескрипторе FAT12.  
 *  
 *  РАЗБОР КОНФИГА (read_file_to_variable)  
 *  --------------------------------------  
 *  Побайтовый парсер ищет в образе диска маркеры вида [CALL], [QTH], [TEXT],  
 *  [START_CW/RTTY/IFKP], [CW_WPM], [RTTY_SPEED], [FREQ_CW], [RTTY_SPACE],  
 *  [RTTY_MARK], [FREQ_IFKP] и раскладывает значения в глобальные String-переменные.  
 *  Частотные поля фильтруются только по цифрам, текст — по печатным символам.  
 *  Здесь же WPM пересчитывается в длину точки CW_DOT_TIME_MS (1200/WPM,  
 *  ограничение 5..50), а бодовая скорость — в RTTY_BIT_TIME_US (1e6/бод).  
 *  
 *  СИНХРОНИЗАЦИЯ С ПК  
 *  ------------------  
 *  msc_flush_cb мгновенно выставляет pc_file_written (аварийный останов текущей  
 *  передачи). check_and_handle_pc_changes в loop() ждёт ~1.5 с тишины после  
 *  последней записи, затем сохраняет диск во Flash, перечитывает настройки и  
 *  «передёргивает» том (setUnitReady false/true), чтобы Windows увидел изменения.  
 *  update_info_config_from_console позволяет менять параметр из UART: находит  
 *  маркер, раздвигает/сдвигает хвост файла (memmove), вписывает значение и  
 *  пересчитывает размер файла в записи FAT12.  
 *  
 *  ГЛОБАЛЬНЫЕ ДАННЫЕ  
 *  -----------------  
 *  my_call/qth/text_variable, my_cw/rtty/ifkp_variable (расписания),  
 *  my_freq_*_var, my_rtty_space/mark_var, my_*_wpm/baud_var — читаются другими  
 *  модулями. CW_DOT_TIME_MS и RTTY_BIT_TIME_US — тайминги для модуляторов.  
 *  pc_file_written — флаг активности ПК для прерывания передач.  
 * ============================================================================  
 */

#include "file_manager.h"
#include "scheduler.h"
#include <Adafruit_TinyUSB.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include "vfo_hardware.h"

// ФИЗИЧЕСКОЕ ОПРЕДЕЛЕНИЕ ОБЪЕКТОВ ДЛЯ ЛИНКОВЩИКА
Adafruit_USBD_MSC usb_msc;
String my_call_variable   = "";
String my_qth_variable    = "";
String my_text_variable   = "";
String my_rtty_variable   = "";
String my_ifkp_variable   = "";
String my_cw_variable     = "";
String my_freq_cw_var     = "";
String my_cw_wpm_var      = "";
String my_rtty_baud_var   = "";
String my_rtty_mark_var   = "";
String my_rtty_shift_var  = ""; 
String my_rtty_invert_var = "";
String my_freq_ifkp_var   = "";
String my_FAT             = "";
String scan_exclude_list  = "";
String scan_result_data   = "";
String my_debug_var       = "";
String s_led_enable       = "";

// Физическое выделение памяти под инженерные переменные железа
// Значения по умолчанию, если теги не найдены
uint32_t CW_DOT_TIME_MS  = 60;              // Время точки в мс (по умолчанию ~20 WPM)
volatile uint32_t RTTY_BIT_TIME_US = 22000; // Время одного бита RTTY в мкс (по умолчанию 45.45 Бод)

// НОВЫЕ ДИНАМИЧЕСКИЕ ИНТЕРВАЛЫ КЛИМАТА
uint32_t climate_sample_interval_min = 10; // По умолчанию замер каждые 10 мин
uint32_t climate_report_interval_min = 60; // По умолчанию таблица каждый час

int subband_pins[4] = {6, 7, 8, 9};  // пины шифра поддиапазона
int pin_freq_out = 10;  // выход DDS-генератора
int pin_amp_act  = 11;  // выход управления усилителем
// пины включения питания модулей
// -1 означает NC (Не назначен / Not Connected)
int pin_pwr_si   = -1;  // генератор SI5351
int pin_pwr_ds   = -1;  // часы RTC
int pin_pwr_bm   = -1;  // климатический датчик
int pin_pwr_dl   = -1;  // дисплей

extern uint16_t rtc_year;  // extern, если объявлены не в этом файле  
extern uint8_t rtc_month, rtc_day, rtc_hour, rtc_min, rtc_sec;

bool debug_flag = false;  // флаг вывода служебных сообщений

// Выделение ОЗУ под таблицу расписания задач
TaskItem beacon_schedule[MAX_SCHEDULE_TASKS];

// Переменные мониторинга текущего состояния флэша (Wear Leveling)
int32_t  current_active_slot = -1; // активный слот
uint32_t current_max_seq     = 0;  // число перезаписей

// Лимиты разгона: паспортные до первого успешного OCTEST/PLLTEST  
uint32_t vfo_max_clk_sys_hz = PASSPORT_CLK_SYS_MAX_HZ;  
uint32_t vfo_max_pll_vco_hz = PASSPORT_PLL_VCO_MAX_HZ;


volatile bool pc_file_written = false;

bool led_enable_flag = true;    // По умолчанию LED-индикация включена
int si5351_clk_tx_out = 0;      // По умолчанию частота TX выдается на CLK0


// Выделение памяти под буфер диска в ОЗУ
alignas(4) static uint8_t ram_disk_buffer[DISK_SIZE_BYTES];

// Переменные времени для отслеживания ПК
volatile uint32_t last_msc_write_time = 0;
bool pc_activity_detected = false;
volatile uint8_t pc_written_regions = 0;  // биты: 0 - системная/FAT/каталог, 1 - INFO.TXT, 2 - SET.TXT, 3 - LOG.TXT

// Колбэки и посредники для TinyUSB MSC

// msc_flush_cb в стеке TinyUSB MSC вызывается не только после реальной записи секторов, 
// но и на SCSI-команду SYNCHRONIZE CACHE, 
// которую Windows посылает регулярно — при монтировании тома, обращении к файлу, фоновом опросе проводника.
void msc_flush_cb(void) {
  last_msc_write_time = millis();
}

int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
  if (lba >= SECTOR_COUNT) return -1;
  memcpy(buffer, &ram_disk_buffer[lba * SECTOR_SIZE], bufsize);
  return bufsize;
}

// callback-функция записи TinyUSB MSC
// вызывается по типу прерывания, когда ПК пишет сектор
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {  
  if (lba >= SECTOR_COUNT) return -1;  
  memcpy(&ram_disk_buffer[lba * SECTOR_SIZE], buffer, bufsize);  
  last_msc_write_time = millis();  
  pc_activity_detected = true;  
  
  // Определяем, какой файл затронула запись  
  if      (lba < 5)                                 pc_written_regions |= 0x01; // бут/FAT/каталог  
  else if (lba < 5 + INFO_CLUSTERS)                 pc_written_regions |= 0x02; // INFO.TXT  
  else if (lba < 5 + INFO_CLUSTERS + SET_CLUSTERS)  pc_written_regions |= 0x04; // SET.TXT  
  else                                              pc_written_regions |= 0x08; // LOG.TXT  
  
  // Прерываем эфир только при изменении данных INFO.TXT (5-24) или SET.TXT (25-44)  
  uint32_t last_lba = lba + (bufsize / SECTOR_SIZE) - 1;  
  if (last_lba >= 5 && lba <= 44) {  
    pc_file_written = true;  // флаг изменения файлов INFO или SET. Выставляем флаг для экстренного останова передачи 
  }  
  return bufsize;  
}



// FAT: время = час<<11 | мин<<5 | сек/2 ; дата = (год-1980)<<9 | месяц<<5 | день  
static uint16_t fat_time_now() {  
  return ((uint16_t)rtc_hour << 11) | ((uint16_t)rtc_min << 5) | (rtc_sec / 2);  
}  
static uint16_t fat_date_now() {  
  uint16_t y = (rtc_year >= 1980) ? (rtc_year - 1980) : 0;  
  return (y << 9) | ((uint16_t)rtc_month << 5) | rtc_day;  
}  
  
// dir_entry_idx: 0=INFO.TXT, 1=SET.TXT, 2=LOG.TXT; with_create — ставить ли дату создания  
static void set_dir_timestamp(uint8_t dir_entry_idx, bool with_create) {  
  uint32_t e = SECTOR_SIZE * 2 + dir_entry_idx * 32;  // корневой каталог  
  uint16_t t = fat_time_now(), d = fat_date_now();  
  if (with_create) {  
    ram_disk_buffer[e + 13] = 0;                                   // fine-res creation (10мс)  
    memcpy(&ram_disk_buffer[e + 14], &t, 2);                       // creation time  
    memcpy(&ram_disk_buffer[e + 16], &d, 2);                       // creation date  
  }  
  memcpy(&ram_disk_buffer[e + 18], &d, 2);                         // last access date  
  memcpy(&ram_disk_buffer[e + 22], &t, 2);                         // write time  
  memcpy(&ram_disk_buffer[e + 24], &d, 2);                         // write date  
}

// функция сохранения ОЗУ во Flash с ротацией по 8 слотам
static void save_ram_to_flash() {
  // Вычисляем индекс следующего слота по кругу и инкрементируем версию
  int32_t next_slot = (current_active_slot + 1) % FLASH_SLOTS;
  uint32_t next_seq = current_max_seq + 1;

  if (current_active_slot == -1) {
    next_slot = 0;
    next_seq = 1;
  }

  uint32_t target_flash_addr = FLASH_TARGET_OFFSET + (next_slot * SLOT_SIZE);

  // Внедряем метаданные износа в скрытые от FAT байты в самом конце буфера ОЗУ (Сектор 255)
  SlotMeta* meta = (SlotMeta*)&ram_disk_buffer[DISK_SIZE_BYTES - sizeof(SlotMeta)];  
  meta->magic = SLOT_MAGIC;  
  meta->seq = next_seq;  
  meta->max_clk_sys_hz = vfo_max_clk_sys_hz;   // лимиты путешествуют вместе со слотом  
  meta->max_pll_vco_hz = vfo_max_pll_vco_hz;

  // Флэш нельзя программировать на разогнанной clk_sys — откатываем частоту на номинал  
  bool was_boosted = clk_boosted;  
  if (was_boosted) vfo_clk_boost_exit(); 

  // Выполняем физическую безопасную запись с отключением прерываний на Core 0
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(target_flash_addr, SLOT_SIZE);
  flash_range_program(target_flash_addr, ram_disk_buffer, DISK_SIZE_BYTES);
  restore_interrupts(ints);
  flash_flush_cache();

  // Возвращаем разгон, если он был активен (сеанс ещё идёт)  
  if (was_boosted) vfo_clk_boost_enter(0); 

  // Обновляем глобальное состояние менеджера только ПОСЛЕ успешного программирования
  current_active_slot = next_slot;
  current_max_seq = next_seq;
}

// Вспомогательная функция для записи 12-битной ячейки в таблицу FAT12
static void set_fat12_entry(uint32_t fat_start_bytes, uint16_t cluster, uint16_t value) {
  uint32_t byte_offset = fat_start_bytes + ((cluster * 3) / 2);
  if (cluster % 2 == 0) {
    ram_disk_buffer[byte_offset] = (uint8_t)(value & 0xFF);
    ram_disk_buffer[byte_offset + 1] = (ram_disk_buffer[byte_offset + 1] & 0xF0) | ((value >> 8) & 0x0F);
  } else {
    ram_disk_buffer[byte_offset] = (ram_disk_buffer[byte_offset] & 0x0F) | ((value << 4) & 0xF0);
    ram_disk_buffer[byte_offset + 1] = (uint8_t)((value >> 4) & 0xFF);
  }
}







/**  
 * @brief Генерация чистого образа диска FAT12 с файлами по умолчанию.  
 *  
 * Полностью очищает RAM-диск @c ram_disk_buffer (128 КБ, 256 секторов  
 * по 512 Б) и вручную размечает его под файловую систему FAT12:  
 * - <b>Сектор 0</b> — загрузочный сектор с BPB: MSDOS5.0, 512 Б/сектор,  
 *   1 сектор/кластер, 1 копия FAT, 48 записей корневого каталога  
 *   (каталог занимает секторы 2..4), метка "PICO DRIVE", сигнатура  
 *   0x55/0xAA — по ней @ref init_file_manager проверяет валидность образа;  
 * - <b>Сектор 1</b> — таблица FAT12: цепочки кластеров для трёх файлов  
 *   связываются динамически через @ref set_fat12_entry с маркером  
 *   конца 0xFFF;  
 * - <b>Сектор 2</b> — корневой каталог: записи INFO.TXT, SET.TXT, LOG.TXT  
 *   со стартовыми кластерами и размерами;  
 * - <b>Секторы 5..24</b> — данные INFO.TXT: шаблон с позывным, QTH,  
 *   частотами и матрицей расписания [TASK_XX];  
 * - <b>Секторы 25..44</b> — данные SET.TXT: содержимое собирается  
 *   динамически из текущих переменных железа (пины, интервалы  
 *   климат-мониторинга, статистика слотов Flash); не назначенные пины  
 *   выводятся как "NC";  
 * - <b>Секторы 45+</b> — данные LOG.TXT: заголовок журнала с версией ПО.  
 *  
 * Завершает работу простановкой временных меток создания/изменения  
 * всем трём записям каталога (@ref set_dir_timestamp) и немедленной  
 * записью образа во Flash через @ref save_ram_to_flash с ротацией  
 * по 8 слотам износа.  
 *  
 * @note Сама функция не вызывает парсеры — обновление глобальных  
 *       переменных из нового образа выполняет вызывающий код  
 *       (@ref init_file_manager, @ref force_reset_to_default_disk)  
 *       через @ref read_and_parse_INFO_txt / @ref read_and_parse_SET_txt.  
 * @warning Функция деструктивна: уничтожает текущее содержимое  
 *          RAM-диска и активного Flash-слота.  
 *  
 * @see init_file_manager(), force_reset_to_default_disk(),  
 *      save_ram_to_flash(), set_fat12_entry(), set_dir_timestamp()  
 */
static void create_default_fat_with_info_file() {
  Serial.println("[Система] Генерируем расширенный диск FAT12 (Каталог: 48 записей)...");
  memset(ram_disk_buffer, 0, DISK_SIZE_BYTES);

  // СЕКТОР 0: Загрузочный сектор (геометрия)
  ram_disk_buffer[0] = 0xEB; ram_disk_buffer[1] = 0x3C; ram_disk_buffer[2] = 0x90; 
  memcpy(&ram_disk_buffer[3], "MSDOS5.0", 8);                                     
  ram_disk_buffer[11] = (uint8_t)(SECTOR_SIZE & 0xFF);         
  ram_disk_buffer[12] = (uint8_t)((SECTOR_SIZE >> 8) & 0xFF);  
  ram_disk_buffer[13] = 1;                                     
  ram_disk_buffer[14] = 1;                                     
  ram_disk_buffer[15] = 0;
  ram_disk_buffer[16] = 1;                                     
  ram_disk_buffer[17] = 48; // 48 записей каталога = 3 сектора (Секторы 2, 3, 4)                        
  ram_disk_buffer[18] = 0;
  ram_disk_buffer[19] = (uint8_t)(SECTOR_COUNT & 0xFF);        
  ram_disk_buffer[20] = (uint8_t)((SECTOR_COUNT >> 8) & 0xFF); 
  ram_disk_buffer[21] = 0xF8;                                  
  ram_disk_buffer[22] = 1;                                     
  ram_disk_buffer[23] = 0;
  ram_disk_buffer[24] = 0x01; ram_disk_buffer[25] = 0x00;     
  ram_disk_buffer[26] = 0x01; ram_disk_buffer[27] = 0x00;     
  ram_disk_buffer[28] = 0x00; ram_disk_buffer[29] = 0x00;     
  ram_disk_buffer[30] = 0x00; ram_disk_buffer[31] = 0x00;     
  ram_disk_buffer[32] = 0x00; ram_disk_buffer[33] = 0x00;     
  ram_disk_buffer[34] = 0x00; ram_disk_buffer[35] = 0x00;
  ram_disk_buffer[36] = 0x80;                                  
  ram_disk_buffer[37] = 0x00;                                  
  ram_disk_buffer[38] = 0x29;                                  
  ram_disk_buffer[39] = 0xDE; ram_disk_buffer[40] = 0xAD; ram_disk_buffer[41] = 0xBE; ram_disk_buffer[42] = 0xEF;
  memcpy(&ram_disk_buffer[43], "PICO DRIVE ", 11);               
  memcpy(&ram_disk_buffer[54], "FAT12   ", 8);                 
  ram_disk_buffer[510] = 0x55; ram_disk_buffer[511] = 0xAA;

  // СЕКТОР 1: Автоматический расчет таблицы FAT12
  uint32_t fat_offset = SECTOR_SIZE * 1;
  ram_disk_buffer[fat_offset + 0] = 0xF8; 
  ram_disk_buffer[fat_offset + 1] = 0xFF; 
  ram_disk_buffer[fat_offset + 2] = 0xFF; 

  // Динамически связываем цепочку кластеров для INFO.TXT
  for (uint16_t i = INFO_FIRST_CLUSTER; i < (INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1); i++) {
    set_fat12_entry(fat_offset, i, i + 1); 
  }
  set_fat12_entry(fat_offset, (INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1), 0xFFF); 

  // Динамически связываем цепочку кластеров для SET.TXT
  for (uint16_t i = SET_FIRST_CLUSTER; i < (SET_FIRST_CLUSTER + SET_CLUSTERS - 1); i++) {
    set_fat12_entry(fat_offset, i, i + 1); 
  }
  set_fat12_entry(fat_offset, (SET_FIRST_CLUSTER + SET_CLUSTERS - 1), 0xFFF); 

  // Динамически связываем цепочку кластеров для LOG.TXT
  for (uint16_t i = LOG_FIRST_CLUSTER; i < (LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1); i++) {
    set_fat12_entry(fat_offset, i, i + 1);
  }
  set_fat12_entry(fat_offset, (LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1), 0xFFF); 

  // СЕКТОР 2: Корневой каталог (Записи по 32 байта)
  uint32_t root_offset = SECTOR_SIZE * 2;

  // 1. Запись для INFO.TXT
  memcpy(&ram_disk_buffer[root_offset + 0], "INFO    ", 8);  
  memcpy(&ram_disk_buffer[root_offset + 8], "TXT", 3);       
  ram_disk_buffer[root_offset + 26] = INFO_FIRST_CLUSTER;    

  // Обновленный дефолтный шаблон INFO.TXT под матричное расписание задач
  const char* default_info_content = 
    "[CALL]=RU0AOG\r\n"
    "[QTH]=NO66FC\r\n"
    "[TEXT]=TESTING BEACON\r\n\r\n"
    "[CW_WPM    ]=20\r\n"
    "[RTTY_SPEED]=45.45\r\n\r\n"
    "[FREQ_CW   ]=3601500\r\n"
    "[RTTY_MARK ]=3601585\r\n"
    "[RTTY_SHIFT]=170\r\n"
    "[RTTY_INVERT]=0\r\n"
    "[FREQ_IFKP ]=3601307\r\n\r\n"
    "=== МАТРИЦА РАСПИСАНИЯ ПЕРЕДАЧ ===\r\n"
    "// ДНИ: 1=Пн, 2=Вт, 3=Ср, 4=Чт, 5=Пт, 6=Сб, 7=Вс, 0=Каждый день\r\n"
    "// МОДЫ: CW, RTTY, IFKP, SEQ (Сквозной цикл CW->RTTY->IFKP)\r\n"
    "// Формат одиночной:     [TASK_01]=ДНИ,ЧЧ:ММ,ЧАСТОТА_ГЦ,МОДА\r\n"
    "// Формат периодической: [TASK_01]=ДНИ,ЧЧ:ММ_СТАРТ/ЧЧ:ММ_КОНЕЦ/ИНТЕРВАЛ,ЧАСТОТА_ГЦ,МОДА\r\n"
    "// Базовое расписание (Ежедневно) ---\r\n"
    "// Плотный дневной цикл каждые 5 минут (с 09:00 до 22:00 ежедневно) ---\r\n"
    "[TASK_01]=0,09:00/22:00/5,3591500,RTTY\r\n"
    "// [TASK_02]=0,15:18,3601585,CW\r\n"
    "// [TASK_03]=0,15:20,3601307,IFKP\r\n"
    "// Примеры:\r\n"
    "// Вечерний плотный цикл каждые 5 минут (с 17:00 до 22:00 ежедневно) ---\r\n"
    "// [TASK_04]=0,17:00/22:00/5,3601500,CW\r\n"
    "// Утренний сквозной трехмодовый цикл каждые 15 минут по будням (Пн-Пт) ---\r\n"
    "// [TASK_05]=12345,08:00/11:30/15,3601000,SEQ\r\n"
    "// Дневная работа на ВЧ-диапазоне 20м (14 МГц) строго по выходным (Сб, Вс) ---\r\n"
    "// [TASK_06]=67,12:00/16:00/30,14095000,IFKP\r\n"
    "// Одиночные ночные запуски в разные дни недели на разных частотах ---\r\n"
    "// [TASK_07]=135,01:30,3601307,IFKP\r\n"
    "// [TASK_08]=246,03:45,7015000,CW\r\n"
    "// [TASK_09]=7,23:59,3601585,RTTY\r\n"
    "[EOF]";
  
  uint32_t info_len = strlen(default_info_content);
  ram_disk_buffer[root_offset + 28] = (uint8_t)(info_len & 0xFF);
  ram_disk_buffer[root_offset + 29] = (uint8_t)((info_len >> 8) & 0xFF);

  // 2. Запись для SET.TXT (Смещение 32)
  uint32_t set_entry_offset = root_offset + 32;
  memcpy(&ram_disk_buffer[set_entry_offset + 0], "SET     ", 8);  
  memcpy(&ram_disk_buffer[set_entry_offset + 8], "TXT", 3);       
  ram_disk_buffer[set_entry_offset + 26] = SET_FIRST_CLUSTER;    

  // Лямбда-помощник для перевода пинов в строку (превращает -1 в NC)
  auto pin_to_str = [](int p) -> String {
    return (p == -1) ? "NC" : String(p);
  };

  // Собираем дефолтное содержимое SET.TXT динамически из текущих переменных железа
  String content = "";
  content.reserve(512);
  content += "=== ИНЖЕНЕРНЫЕ НАСТРОЙКИ МАЯКА ===\r\n";
  content += "[PIN_FREQ_OUT ]=" + pin_to_str(pin_freq_out) + "\r\n";
  content += "[PIN_AMP_ACT  ]=" + pin_to_str(pin_amp_act) + "\r\n\r\n";
  content += "// Выходной канал частоты передачи Si5351: 0 = CLK0, 1 = CLK1, 2 = CLK2\r\n";
  content += "[SI5351_CLK_OUT]=" + String(si5351_clk_tx_out) + "\r\n\r\n";
  content += "// Пины кода поддиапазона (4 пина)\r\n";
  content += "[PIN_SUBBAND_0]=" + pin_to_str(subband_pins[0]) + "\r\n";
  content += "[PIN_SUBBAND_1]=" + pin_to_str(subband_pins[1]) + "\r\n";
  content += "[PIN_SUBBAND_2]=" + pin_to_str(subband_pins[2]) + "\r\n";
  content += "[PIN_SUBBAND_3]=" + pin_to_str(subband_pins[3]) + "\r\n\r\n";
  content += "// Пины питания шины (NC если не назначены)\r\n";
  content += "[PIN_PWR_SI   ]=" + pin_to_str(pin_pwr_si) + "\r\n";
  content += "[PIN_PWR_DS   ]=" + pin_to_str(pin_pwr_ds) + "\r\n";
  content += "[PIN_PWR_BM   ]=" + pin_to_str(pin_pwr_bm) + "\r\n";
  content += "[PIN_PWR_DL   ]=" + pin_to_str(pin_pwr_dl) + "\r\n\r\n";
  content += "// Дополнительные исключения из сканирования шин\r\n";
  content += "[SCAN_EXCLUDE ]=16,23,24,25\r\n\r\n";
  content += "// Режим отладки: 1 — служебные сообщения в Serial, 0 — выкл\r\n";
  content += "[DEBUG]=" + String(debug_flag ? 1 : 0) + "\r\n\r\n";   // 0/1 — режим отладки
  content += "// Управление светодиодной индикацией: 1 — включена, 0 — выключена\r\n";
  content += "[LED_ENABLE]=" + String(led_enable_flag ? 1 : 0) + "\r\n\r\n";
  // СОХРАНЕНИЕ ТЕКУЩИХ ИНТЕРВАЛОВ ПРИ ПЕРЕЗАПИСИ ИЛИ ИНИЦИАЛИЗАЦИИ ДИСКА
  content += "=== ПЕРИОДИЧНОСТЬ КЛИМАТИЧЕСКОГО МОНИТОРИНГА ===\r\n";
  content += "[CLIM_SAMPLE_MIN]=" + String(climate_sample_interval_min) + "\r\n";
  content += "[CLIM_REPORT_MIN]=" + String(climate_report_interval_min) + "\r\n\r\n";
  content += "=== СТАТИСТИКА ИЗНОСА ФЛЭШ-ПАМЯТИ ===\r\n";
  content += "[FLASH_SLOT   ]=" + String(current_active_slot != -1 ? current_active_slot : 0) + "\r\n";
  content += "[FLASH_SEQ    ]=" + String(current_max_seq != 0 ? current_max_seq : 1) + "\r\n\r\n";
  content += "=== УСТРОЙСТВА НА ШИНЕ I2C ===\r\n";
  content += "[SCAN_RESULT]\r\nСканирование не проводилось.\r\n\r\n";
  content += "[EOF]";
  
  uint32_t set_len = content.length();
  ram_disk_buffer[set_entry_offset + 28] = (uint8_t)(set_len & 0xFF);
  ram_disk_buffer[set_entry_offset + 29] = (uint8_t)((set_len >> 8) & 0xFF);

  // 3. Запись для LOG.TXT (Смещение 64)
  uint32_t log_entry_offset = root_offset + 64;
  memcpy(&ram_disk_buffer[log_entry_offset + 0], "LOG     ", 8); 
  memcpy(&ram_disk_buffer[log_entry_offset + 8], "TXT", 3);      
  ram_disk_buffer[log_entry_offset + 26] = LOG_FIRST_CLUSTER;   

  String default_log_content = "           === ЖУРНАЛ РАБОТЫ МАЯКА ===\r\n        Версия ПО v." + BCN_VER + " от " + BCN_DAT + "\r\n";
  uint32_t log_len = default_log_content.length();
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(log_len & 0xFF);
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((log_len >> 8) & 0xFF);

  // === СМЕЩЕНИЕ ОБЛАСТИ ДАННЫХ НА СЕКТОР 5 ===
  // Сектор 5: Данные INFO.TXT
  uint32_t info_data_offset = SECTOR_SIZE * 5; // Сектор 5
  memcpy(&ram_disk_buffer[info_data_offset], default_info_content, info_len);

  // Сектор 25: Данные SET.TXT (5 + 20)
  uint32_t set_data_offset = SECTOR_SIZE * 25; // Сектор 25 (5 + 20)
  memcpy(&ram_disk_buffer[set_data_offset], content.c_str(), set_len);

  // Сектор 45: Данные LOG.TXT (5 + 20 + 20)
  uint32_t log_data_offset = SECTOR_SIZE * 45; // Сектор 45 (5 + 20 + 20)
  memcpy(&ram_disk_buffer[log_data_offset], default_log_content.c_str(), log_len);

  set_dir_timestamp(0, true);   // INFO.TXT — создание + изменение  
  set_dir_timestamp(1, true);   // SET.TXT  
  set_dir_timestamp(2, true);   // LOG.TXT

  save_ram_to_flash();
  Serial.println("[Система] Структура диска обновлена: INFO (10Кб), SET (10Кб) и LOG (100Кб) готовы на Секторе 5!");
}



// для парсинга SET.TXT
// Вспомогательный инлайн для обработки значений пинов (парсит числа или возвращает -1 для NC)
static inline int parse_pin_value(const String& val) {
  String tmp = val;
  tmp.trim();
  if (tmp.equalsIgnoreCase("NC") || tmp.length() == 0) return -1;
  return tmp.toInt();
}



/**  
 * @brief Побайтовый парсер файла INFO.TXT в глобальные настройки маяка.  
 *  
 * Сканирует область данных INFO.TXT в RAM-диске (секторы 5..24,  
 * `INFO_CLUSTERS * SECTOR_SIZE` байт) и извлекает все маркеры вида  
 * `[ТЕГ]=значение`. Тег распознаётся только если '[' стоит в начале  
 * файла или после перевода строки/непечатного символа — так текст в  
 * середине строки не ломает разбор. Имя тега ищется динамически до  
 * ']', пробелы выравнивания внутри тега игнорируются.  
 *  
 * Извлекаемые теги:  
 * - текстовые: `CALL`, `QTH`, `TEXT`, `CW`, `RTTY`, `IFKP`  
 *   (расписания-строки) — фильтрация по печатным символам (>= 32);  
 * - числовые: `FREQ_CW`, `FREQ_IFKP`, `RTTY_MARK`, `RTTY_SHIFT`,  
 *   `RTTY_INVERT` — фильтрация только по цифрам;  
 * - скорости: `CW_WPM`, `RTTY_SPEED`;  
 * - матрица расписания `TASK_XX` (до MAX_SCHEDULE_TASKS задач):  
 *   CSV-формат `ДНИ,ВРЕМЯ,ЧАСТОТА,МОДА`, где ДНИ — посимвольная маска  
 *   1..7 (0 = каждый день), ВРЕМЯ — одиночное `ЧЧ:ММ` или периодическое  
 *   `ЧЧ:ММ_СТАРТ/ЧЧ:ММ_КОНЕЦ/ИНТЕРВАЛ`, МОДА — CW/RTTY/IFKP/SEQ.  
 *   Результат раскладывается в @ref beacon_schedule.  
 *  
 * После разбора пересчитывает производные тайминги:  
 * - `CW_DOT_TIME_MS = 1200 / WPM` (WPM зажимается в 5..50,  
 *   при отсутствии — 20 WPM);  
 * - `RTTY_BIT_TIME_US = 1e6 / baud`.  
 *  
 * Перед разбором полностью обнуляет все целевые String-переменные и  
 * массив задач (`active = false`) — функция безопасна для повторного  
 * вызова при правках файла с ПК. Вызывается из @ref init_file_manager,  
 * @ref check_and_handle_pc_changes и при мягком рестарте в `loop()`.  
 *  
 * @note Значение тега читается строго до конца строки (`\n`, `\r` или  
 *       следующего '['); `=` после ']' пропускается опционально.  
 * @note Невалидные строки TASK (нет 3 запятых, неизвестная мода,  
 *       переполнение MAX_SCHEDULE_TASKS) молча пропускаются.  
 *  
 * @see read_and_parse_SET_txt(), init_file_manager(),  
 *      check_and_handle_pc_changes(), TaskItem, beacon_schedule  
 */
void read_and_parse_INFO_txt() {
  flash_flush_cache();
  
  // 1. Полностью обнуляем ВСЕ строки перед чтением
  my_call_variable = "";  my_qth_variable  = "";  my_text_variable = "";
  my_rtty_variable = "";  my_ifkp_variable = "";  my_cw_variable = "";    
  my_freq_cw_var = "";    my_freq_ifkp_var = "";  my_rtty_mark_var = "";  
  my_rtty_shift_var = ""; my_rtty_invert_var = ""; my_cw_wpm_var = "";    my_rtty_baud_var = "";

  // Обнуляем старый массив матричного расписания задач
  for (int t = 0; t < MAX_SCHEDULE_TASKS; t++) {
    beacon_schedule[t].active = false;
  }

  // Границы сканирования файла INFO.TXT (Секторы 5..24)
  uint32_t scan_start = 5 * SECTOR_SIZE;
  uint32_t scan_end   = scan_start + (INFO_CLUSTERS * SECTOR_SIZE);
  int task_counter = 0;

  for (uint32_t i = scan_start; i < scan_end - 15; i++) {
    if (ram_disk_buffer[i] == '[' &&   
    (i == scan_start || ram_disk_buffer[i-1] == '\n' ||   
     ram_disk_buffer[i-1] == '\r' || (uint8_t)ram_disk_buffer[i-1] < 32 ||   
     (uint8_t)ram_disk_buffer[i-1] >= 0x80)) {
          int32_t start_idx = -1;  
          String* target_str = nullptr;  
          bool is_task_line = false;  
      
          // Динамически ищем закрывающую скобку ']' — пробелы выравнивания в теге не ломают разбор  
          uint32_t close_bracket_idx = 0;  
          for (uint32_t k = i + 1; k < i + 20 && k < scan_end; k++) {  
            if (ram_disk_buffer[k] == ']') { close_bracket_idx = k; break; }  
          }  
          if (close_bracket_idx == 0) continue;  // ']' не найдена — не тег, идём дальше  
      
          // Имя тега между '[' и ']', без пробелов по краям  
          String tag = "";  
          for (uint32_t k = i + 1; k < close_bracket_idx; k++) {  
            char c = (char)ram_disk_buffer[k];  
            if (c != ' ') tag += c;  
          }  
      
          // --- БЛОК А: Одиночные текстовые и частотные маркеры ---  
          if      (tag == "CALL")        target_str = &my_call_variable;  
          else if (tag == "QTH")         target_str = &my_qth_variable;  
          else if (tag == "TEXT")        target_str = &my_text_variable;  
          else if (tag == "CW_WPM")      target_str = &my_cw_wpm_var;  
          else if (tag == "RTTY_SPEED")  target_str = &my_rtty_baud_var;  
          else if (tag == "FREQ_CW")     target_str = &my_freq_cw_var;  
          else if (tag == "RTTY_MARK")   target_str = &my_rtty_mark_var;  
          else if (tag == "RTTY_SHIFT")  target_str = &my_rtty_shift_var;  
          else if (tag == "RTTY_INVERT") target_str = &my_rtty_invert_var;  
          else if (tag == "FREQ_IFKP")   target_str = &my_freq_ifkp_var;  
          else if (tag == "CW")          target_str = &my_cw_variable;    // НОВОЕ  
          else if (tag == "RTTY")        target_str = &my_rtty_variable;  // НОВОЕ  
          else if (tag == "IFKP")        target_str = &my_ifkp_variable;  // НОВОЕ

          // --- БЛОК Б: Матрица расписания [TASK_XX] ---  
          else if (tag.startsWith("TASK_")) {  
            is_task_line = true;  
          }  
      
          if (target_str != nullptr || is_task_line) {  
            start_idx = close_bracket_idx + 1;  // значение начинается сразу после ']'  
          }

      // 2. Выкусываем значение тега строго до конца строки
      if (start_idx != -1) {
        if (ram_disk_buffer[start_idx] == '=') start_idx++;

        String value = "";
        value.reserve(64);

        for (uint32_t j = start_idx; j < scan_end; j++) {
          char c = (char)ram_disk_buffer[j];
          if (c == '\n' || c == '\r' || c == '[') { i = j - 1; break; }
          
          if (!is_task_line && (target_str == &my_freq_ifkp_var || target_str == &my_rtty_mark_var || 
              target_str == &my_rtty_shift_var || target_str == &my_rtty_invert_var || target_str == &my_freq_cw_var)) {
            if (c >= '0' && c <= '9') value += c;
          } else {
            if (c >= 32) value += c;
          }
        }
        value.trim();

        // Распределяем собранную строку
        if (is_task_line) {
          // Выполняем CSV-парсинг строки задачи: ДНИ,ВРЕМЯ,ЧАСТОТА,МОДА
          int comma1 = value.indexOf(',');
          int comma2 = value.indexOf(',', comma1 + 1);
          int comma3 = value.indexOf(',', comma2 + 1);
          
          if (comma1 != -1 && comma2 != -1 && comma3 != -1 && task_counter < MAX_SCHEDULE_TASKS) {
            String s_days = value.substring(0, comma1);
            String s_time = value.substring(comma1 + 1, comma2);
            String s_freq = value.substring(comma2 + 1, comma3);
            String s_mode = value.substring(comma3 + 1);
            s_days.trim(); s_time.trim(); s_freq.trim(); s_mode.trim();

            TaskItem& task = beacon_schedule[task_counter];
            task.days = 0; // Изначально обнуляем маску
            
            if (!s_days.equals("0")) {
              // Посимвольно разбираем строку (например, "12345") и взводим биты
              for (size_t ch = 0; ch < s_days.length(); ch++) {
                char d_char = s_days[ch];
                if (d_char >= '1' && d_char <= '7') {
                  uint8_t day_num = d_char - '0';
                  task.days |= (1 << day_num); // Взводим нужный бит (1..7)
                }
              }
            }
            task.freq_hz = strtoul(s_freq.c_str(), NULL, 10);

            if (s_mode.equalsIgnoreCase("CW"))        task.mode = MODE_CW;
            else if (s_mode.equalsIgnoreCase("RTTY"))  task.mode = MODE_RTTY;
            else if (s_mode.equalsIgnoreCase("IFKP"))  task.mode = MODE_IFKP;
            else if (s_mode.equalsIgnoreCase("SEQ"))   task.mode = MODE_SEQ;
            else continue;

            int slash1 = s_time.indexOf('/');
            if (slash1 == -1) { // Одиночная задача "15:15"
              int colon = s_time.indexOf(':');
              if (colon != -1) {
                task.start_hour = s_time.substring(0, colon).toInt();
                task.start_min  = s_time.substring(colon + 1).toInt();
                task.end_hour   = task.start_hour;
                task.end_min    = task.start_min;
                task.interval_min = 0;
              }
            } else { // Периодическая задача "17:00/22:00/5"
              int slash2 = s_time.indexOf('/', slash1 + 1);
              if (slash2 != -1) {
                String s_start = s_time.substring(0, slash1);
                String s_end   = s_time.substring(slash1 + 1, slash2);
                task.interval_min = s_time.substring(slash2 + 1).toInt();

                int colon1 = s_start.indexOf(':');
                int colon2 = s_end.indexOf(':');
                if (colon1 != -1 && colon2 != -1) {
                  task.start_hour = s_start.substring(0, colon1).toInt();
                  task.start_min  = s_start.substring(colon1 + 1).toInt();
                  task.end_hour   = s_end.substring(0, colon2).toInt();
                  task.end_min    = s_end.substring(colon2 + 1).toInt();
                }
              }
            }
            task.active = true;
            task_counter++;
          }
        } else if (target_str != nullptr) {
          *target_str = value;
        }
      }
    }
  }
  
  // Рассчитываем длительность точки CW из WPM
  if (my_cw_wpm_var.length() > 0) {
    int wpm = my_cw_wpm_var.toInt();
    if (wpm < 5)  wpm = 5;   
    if (wpm > 50) wpm = 50; 
    my_cw_wpm_var = String(wpm); 
    CW_DOT_TIME_MS = 1200 / wpm; 
  } else {
    CW_DOT_TIME_MS = 60; 
    my_cw_wpm_var = "20"; 
  }

  // Расчет длительности бита RTTY
  if (my_rtty_baud_var.length() > 0) {
    float baud = my_rtty_baud_var.toFloat(); 
    if (baud > 0.0f) {
      RTTY_BIT_TIME_US = (uint32_t)(1000000.0f / baud);
    }
  }

  if (debug_flag) {
    Serial.println("[Система] парсинг INFO.TXT");
  }

}


/**  
 * @brief Парсер инженерных настроек файла SET.TXT.  
 *  
 * Побайтово сканирует область данных SET.TXT в RAM-диске (сектор 25 +  
 * `SET_CLUSTERS * SECTOR_SIZE` байт) и извлекает теги вида `[ТЕГ]=значение`.  
 * Логика сканирования идентична @ref read_and_parse_INFO_txt: тег  
 * распознаётся только если '[' стоит в начале области или после  
 * перевода строки/непечатного символа; имя тега ищется динамически  
 * до ']', пробелы выравнивания внутри скобок игнорируются; значение  
 * читается до конца строки с фильтрацией непечатных символов (< 32).  
 *  
 * Распознаваемые теги и их назначение:  
 * - `PIN_FREQ_OUT` — GPIO выхода ВЧ-генератора PIO (дефолт 14);  
 * - `PIN_AMP_ACT` — GPIO активации усилителя мощности (дефолт 15);  
 * - `PIN_SUBBAND_0..3` — 4 пина кода поддиапазона (дефолт 2+k);  
 * - `PIN_PWR_SI/DS/BM/DL` — пины питания модулей Si5351, RTC-часов,  
 *   климат-датчика и дисплея (дефолт -1 = NC);  
 * - `SI5351_CLK_OUT` — выходной канал Si5351 для TX (0..2, дефолт CLK0);  
 * - `LED_ENABLE` — флаг индикаторного светодиода (дефолт включён);  
 * - `SCAN_EXCLUDE` — список I2C-адресов, исключённых из сканера шины;  
 * - `DEBUG` — флаг служебных сообщений (@c debug_flag);  
 * - `CLIM_SAMPLE_MIN`, `CLIM_REPORT_MIN` — интервалы климат-  
 *   мониторинга в минутах (защита от 0 → 1).  
 *  
 * Значения пинов разбираются через @ref parse_pin_value: строка "NC"  
 * или пустое значение превращаются в -1 (цепь не подключена).  
 * Ключевая особенность — защита от сброса в 0: если тег отсутствует  
 * или строка пустая, переменной присваивается жёстко заданный дефолт,  
 * а не ноль, поэтому частично заполненный SET.TXT не ломает схему.  
 *  
 * Перед разбором обнуляет промежуточные строки и глобальные  
 * `scan_exclude_list`, `scan_result_data`, `my_debug_var` — функция  
 * безопасна для повторного вызова при правках файла с ПК.  
 *  
 * Вызывается из @ref init_file_manager (старт), @ref force_reset_to_default_disk  
 * и @ref check_and_handle_pc_changes при обнаружении записи в SET.TXT.  
 *  
 * @note В отличие от парсера INFO.TXT, здесь нет посимвольной цифровой  
 *       фильтрации значений — собираются все печатные символы,  
 *       поэтому "NC" доходит до `parse_pin_value` в исходном виде.  
 * @warning Пины применяются только до следующей инициализации  
 *          периферии; изменение `PIN_FREQ_OUT`/`PIN_AMP_ACT` требует  
 *          рестарта, т.к. GPIO конфигурируются в setup-фазе.  
 *  
 * @see read_and_parse_INFO_txt(), parse_pin_value(), init_file_manager(),  
 *      check_and_handle_pc_changes()  
 */
void read_and_parse_SET_txt() {
  flash_flush_cache();
  
  String s_freq_out = "", s_amp_act = "";
  String s_subband[4] = {"", "", "", ""};
  String s_pwr_si = "", s_pwr_ds = "", s_pwr_bm = "", s_pwr_dl = "";
  scan_exclude_list = "";
  scan_result_data  = "";
  my_debug_var      = "";
  s_led_enable = "";
  String s_si_clk = "";
  String s_clim_sample = "", s_clim_report = "";

  uint32_t scan_start = 25 * SECTOR_SIZE; // Сектор 25
  uint32_t scan_end   = scan_start + (SET_CLUSTERS * SECTOR_SIZE);

  for (uint32_t i = scan_start; i < scan_end - 15; i++) {
    if (ram_disk_buffer[i] == '[' &&   
    (i == scan_start || ram_disk_buffer[i-1] == '\n' ||   
     ram_disk_buffer[i-1] == '\r' || (uint8_t)ram_disk_buffer[i-1] < 32 ||   
     (uint8_t)ram_disk_buffer[i-1] >= 0x80)) {
          int32_t start_idx = -1;  
          String* target_str = nullptr;  
      
          // Динамически ищем закрывающую скобку ']' — пробелы выравнивания в теге не ломают разбор  
          uint32_t close_bracket_idx = 0;  
          for (uint32_t k = i + 1; k < i + 20 && k < scan_end; k++) {  
            if (ram_disk_buffer[k] == ']') { close_bracket_idx = k; break; }  
          }  
          if (close_bracket_idx == 0) continue;  // ']' не найдена — не тег, идём дальше  
      
          // Имя тега между '[' и ']', без пробелов по краям  
          String tag = "";  
          for (uint32_t k = i + 1; k < close_bracket_idx; k++) {  
            char c = (char)ram_disk_buffer[k];  
            if (c != ' ') tag += c;  
          }

          if      (tag == "PIN_FREQ_OUT")    target_str = &s_freq_out;  
          else if (tag == "PIN_AMP_ACT")     target_str = &s_amp_act;
          else if (tag == "SI5351_CLK_OUT")  target_str = &s_si_clk;
          else if (tag == "LED_ENABLE")      target_str = &s_led_enable;
          else if (tag == "PIN_SUBBAND_0")   target_str = &s_subband[0];  
          else if (tag == "PIN_SUBBAND_1")   target_str = &s_subband[1];  
          else if (tag == "PIN_SUBBAND_2")   target_str = &s_subband[2];  
          else if (tag == "PIN_SUBBAND_3")   target_str = &s_subband[3];  
          else if (tag == "PIN_PWR_SI")      target_str = &s_pwr_si;  
          else if (tag == "PIN_PWR_DS")      target_str = &s_pwr_ds;  
          else if (tag == "PIN_PWR_BM")      target_str = &s_pwr_bm;  
          else if (tag == "PIN_PWR_DL")      target_str = &s_pwr_dl;  
          else if (tag == "SCAN_EXCLUDE")    target_str = &scan_exclude_list;  
          else if (tag == "DEBUG")           target_str = &my_debug_var;
          else if (tag == "CLIM_SAMPLE_MIN") target_str = &s_clim_sample;
          else if (tag == "CLIM_REPORT_MIN") target_str = &s_clim_report;

          if (target_str != nullptr) {  
            start_idx = close_bracket_idx + 1;  // значение начинается сразу после ']'  
          }

      // Выкусываем значение строго до конца строки
      if (start_idx != -1 && target_str != nullptr) {
        target_str->reserve(64);
        if (ram_disk_buffer[start_idx] == '=') start_idx++;

        for (uint32_t j = start_idx; j < scan_end; j++) {
          char c = (char)ram_disk_buffer[j];
          if (c == '\n' || c == '\r' || c == '[') {
            i = j - 1;
            break;
          }
          if (c >= 32) *target_str += c;
        }
        target_str->trim();
      }
    }
  }

  debug_flag = (my_debug_var.toInt() != 0);
  // Назначаем дефолты жестко в коде, если файлы пустые или теги не прочитались
  pin_freq_out = (s_freq_out.length() > 0) ? parse_pin_value(s_freq_out) : 14;
  pin_amp_act  = (s_amp_act.length() > 0)  ? parse_pin_value(s_amp_act)  : 15;
  
  for (int k = 0; k < 4; k++) {
    subband_pins[k] = (s_subband[k].length() > 0) ? parse_pin_value(s_subband[k]) : (2 + k);
  }
  
  pin_pwr_si = (s_pwr_si.length() > 0) ? parse_pin_value(s_pwr_si) : -1;
  pin_pwr_ds = (s_pwr_ds.length() > 0) ? parse_pin_value(s_pwr_ds) : -1;
  pin_pwr_bm = (s_pwr_bm.length() > 0) ? parse_pin_value(s_pwr_bm) : -1;
  pin_pwr_dl = (s_pwr_dl.length() > 0) ? parse_pin_value(s_pwr_dl) : -1;
  led_enable_flag = (s_led_enable.length() > 0) ? (s_led_enable.toInt() != 0) : true;
  int parsed_clk = (s_si_clk.length() > 0) ? s_si_clk.toInt() : 0;
  si5351_clk_tx_out = (parsed_clk >= 0 && parsed_clk <= 2) ? parsed_clk : 0;
  if (s_clim_sample.length() > 0) {
    uint32_t val = s_clim_sample.toInt();
    climate_sample_interval_min = (val > 0) ? val : 1; // Защита от 0
  }
  if (s_clim_report.length() > 0) {
    uint32_t val = s_clim_report.toInt();
    climate_report_interval_min = (val > 0) ? val : 1; // Защита от 0
  }
  if (debug_flag) {
    Serial.println("[Система] парсинг SET.TXT");
  }

}


// Функция вывода текущих настроек
void print_current_settings() {
  my_cw_variable   = fmt_next_start(get_next_start_minute(MODE_CW));  
  my_rtty_variable = fmt_next_start(get_next_start_minute(MODE_RTTY));
  my_ifkp_variable = fmt_next_start(get_next_start_minute(MODE_IFKP));
  Serial.println("=== ТЕКУЩИЕ НАСТРОЙКИ РАДИОМАЯКА ===");
  Serial.print("Позывной [CALL      ]: "); Serial.println(my_call_variable.length() > 0 ? my_call_variable : "Не задан");
  Serial.print("Локатор  [QTH       ]: "); Serial.println(my_qth_variable.length()  > 0 ? my_qth_variable  : "Не задан");
  Serial.print("Текст    [TEXT      ]: "); Serial.println(my_text_variable.length() > 0 ? my_text_variable : "Не задан");
  Serial.print("Таймер   [START_CW  ]: "); Serial.println(my_cw_variable.length()   > 0 ? my_cw_variable   : "Не задан");
  Serial.print("Таймер   [START_RTTY]: "); Serial.println(my_rtty_variable.length() > 0 ? my_rtty_variable : "Не задан");
  Serial.print("Таймер   [START_IFKP]: "); Serial.println(my_ifkp_variable.length() > 0 ? my_ifkp_variable : "Не задан");
  Serial.print("Скорость  CW [CW_WPM]: "); Serial.print(my_cw_wpm_var.length() > 0 ? my_cw_wpm_var : "20"); Serial.print(" WPM (Длина точки: "); Serial.print(CW_DOT_TIME_MS); Serial.println(" ms)");
  Serial.print("Скорость [RTTY_SPEED]: "); Serial.print(my_rtty_baud_var.length() > 0 ? my_rtty_baud_var : "45.45"); Serial.print(" Baud (Длина бита: "); Serial.print(RTTY_BIT_TIME_US/1000); Serial.println(" ms)");
  Serial.print("Частота  [FREQ_CW   ]: "); Serial.print(my_freq_cw_var.length() > 0 ? my_freq_cw_var : "3601000 (Резерв)"); Serial.println(" Hz");
  Serial.print("Частота  [RTTY_MARK ]: "); Serial.print(my_rtty_mark_var); Serial.println(" Hz");
  Serial.print("Сдвиг    [RTTY_SHIFT]: "); Serial.print(my_rtty_shift_var.length() > 0 ? my_rtty_shift_var : "170"); Serial.println(" Hz");
  Serial.print("Инверсия [RTTY_INV  ]: "); Serial.println(my_rtty_invert_var == "1" ? "ВКЛЮЧЕНА (Mark < Space)" : "ВЫКЛЮЧЕНА (Mark > Space)");
  Serial.print("Частота  [FREQ_IFKP ]: "); Serial.print(my_freq_ifkp_var); Serial.println(" Hz");

  // Вывод аппаратной конфигурации пинов из SET.TXT
  Serial.println("--- Аппаратная конфигурация (SET.TXT) ---");
  Serial.print("Пин ВЧ-выхода   (FREQ_OUT): "); Serial.println(pin_freq_out);
  Serial.print("Выход частоты TX  (Si5351): CLK"); Serial.println(si5351_clk_tx_out);
  Serial.print("Пин активации УМ (AMP_ACT): "); Serial.println(pin_amp_act);
  Serial.print("Пины поддиапазонов        : ");
  for (int k = 0; k < 4; k++) {
    Serial.print(subband_pins[k]); if (k < 3) Serial.print(", ");
  }
  Serial.println("");
  
  auto print_pwr_pin = [](const char* label, int pin) {
    Serial.print("Питание устройства ["); Serial.print(label); Serial.print("]   : ");
    if (pin == -1) Serial.println("NC (Не назначен)");
    else Serial.println(pin);
  };
  print_pwr_pin("SI", pin_pwr_si);
  print_pwr_pin("DS", pin_pwr_ds);
  print_pwr_pin("BM", pin_pwr_bm);
  print_pwr_pin("DL", pin_pwr_dl);
  // Корректное распознавание флага NC (Not Connected) для списка исключений
  Serial.print("Исключения сканера шины   : "); 
  if (scan_exclude_list.length() == 0 || scan_exclude_list.equalsIgnoreCase("NC")) {
    Serial.println("NC (Не назначены)");
  } else {
    Serial.println(scan_exclude_list);
  }


  // Диагностика износа ячеек и активных слотов памяти
  Serial.println("--- Статистика износа флеш-памяти ---");
  Serial.print("Активный слот флеша  : "); Serial.println(current_active_slot != -1 ? String(current_active_slot) : "Не определен");
  Serial.print("Счетчик записей (seq): "); Serial.println(current_max_seq);
  Serial.print("Физический адрес флеш: 0x"); Serial.println(0x10000000 + FLASH_TARGET_OFFSET + (current_active_slot * SLOT_SIZE), HEX);

  Serial.println("=====================================");
}




// Регистрирует дескрипторы Mass Storage в стек TinyUSB до начала энумерации хостом
void init_usb_msc_interface() {
  usb_msc.setCapacity(SECTOR_COUNT, SECTOR_SIZE);
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setID("RU0AOG", "Beacon", "2.0");
  usb_msc.begin();
  usb_msc.setUnitReady(false); // Диск пока "не вставлен", так как память из Flash еще не считана
}


// инициализация файлового менеджера со сканированием износа слотов
void init_flash_disk() {
  // Сигнализируем Windows, что медианоситель успешно вставлен и готов к работе
  usb_msc.setUnitReady(true);
}





/**  
 * @brief Инициализация файлового менеджера: загрузка в RAM-диск образа диска из Flash.  
 *  
 * Выполняет двухэтапное восстановление RAM-диска при старте устройства:  
 *  
 * -# <b>Сканирование слотов (wear leveling).</b> Перебирает все  
 *    @c FLASH_SLOTS (8) областей по 128 КБ во Flash начиная с  
 *    @c FLASH_TARGET_OFFSET. Для каждого слота проверяет три признака  
 *    валидности: магическое число @c SlotMeta::magic (@c SLOT_MAGIC)  
 *    в конце сектора 255, сигнатуру FAT12 0x55/0xAA в конце  
 *    загрузочного сектора и число записей корневого каталога = 48  
 *    (байт 17 BPB). Среди валидных слотов выбирается слот с  
 *    максимальным счётчиком версий @c seq — он запоминается в  
 *    @c current_active_slot и @c current_max_seq.  
 *  
 * -# <b>Загрузка или регенерация.</b> Если валидный слот найден —  
 *    образ целиком копируется в @c ram_disk_buffer, после чего  
 *    проверяются первые байты имён всех трёх файлов в корневом  
 *    каталоге (сектор 2): 0x00 или 0xE5 означает, что файл был стёрт  
 *    операционной системой ПК — тогда диск принудительно пересоздаётся  
 *    вызовом @ref create_default_fat_with_info_file. Если ни один слот  
 *    не прошёл проверку — диск генерируется по умолчанию.  
 *  
 * Завершает работу полным разбором конфигурации:  
 * @ref read_and_parse_INFO_txt (позывной, частоты, расписание) и  
 * @ref read_and_parse_SET_txt (пины железа, флаги). Текстовый статус  
 * результата сохраняется в строку @c my_FAT для вывода в Serial.  
 *  
 * @note Вызывается из `setup()` один раз, после  
 *       @ref init_usb_msc_interface (дескрипторы MSC уже  
 *       зарегистрированы, но том ещё «не вставлен» —  
 *       `setUnitReady(false)`); пометка «диск вставлен» выполняется  
 *       позже в @ref init_flash_disk.  
 * @note Чтение Flash выполняется по абсолютным XIP-адресам  
 *       (0x10000000 + offset) через `memcpy` напрямую из памяти —  
 *       без загрузки во временные буферы.  
 *  
 * @see create_default_fat_with_info_file(), save_ram_to_flash(),  
 *      read_and_parse_INFO_txt(), read_and_parse_SET_txt(),  
 *      init_usb_msc_interface(), init_flash_disk(), SlotMeta  
 */
void init_file_manager() {
  flash_flush_cache();

  current_active_slot = -1;
  current_max_seq = 0;
  bool slot_found = false;

  SlotMeta checked_meta;

  // Шаг 1: Сканируем все 8 слотов по 128 КБ в поисках максимального валидного seq
  for (int i = 0; i < FLASH_SLOTS; i++) {
    uint32_t flash_addr_abs = 0x10000000 + FLASH_TARGET_OFFSET + (i * SLOT_SIZE);
    
    // Считываем структуру метаданных из самого конца проверяемого слота
    uint32_t meta_offset_abs = flash_addr_abs + DISK_SIZE_BYTES - sizeof(SlotMeta);
    memcpy(&checked_meta, (const void*)meta_offset_abs, sizeof(SlotMeta));

    // Проверяем валидность сигнатур FAT12 (0x55/0xAA) в конце 0-го сектора этого слота
    uint32_t boot_sig_offset_abs = flash_addr_abs + 510;
    uint8_t sig_low = *(const uint8_t*)(boot_sig_offset_abs);
    uint8_t sig_high = *(const uint8_t*)(boot_sig_offset_abs + 1);

    // Проверка количества записей в каталоге (смещение 17 в бут-секторе BPB)
    uint8_t root_entries_count = *(const uint8_t*)(flash_addr_abs + 17);

    if (checked_meta.magic == SLOT_MAGIC && sig_low == 0x55 && sig_high == 0xAA && root_entries_count == 48) {  
      if (checked_meta.seq >= current_max_seq) {  
        current_max_seq = checked_meta.seq;  
        current_active_slot = i;  
        slot_found = true;  
        // Лимиты из старых образов читаются как 0xFFFFFFFF (стёртая flash) или 0 —  
        // это невалидно, тогда остаются паспортные  
        if (checked_meta.max_clk_sys_hz >= PASSPORT_CLK_SYS_MAX_HZ &&  
            checked_meta.max_clk_sys_hz <= 1000000000UL) {  
          vfo_max_clk_sys_hz = checked_meta.max_clk_sys_hz;  
        }  
        if (checked_meta.max_pll_vco_hz >= PASSPORT_PLL_VCO_MAX_HZ &&  
            checked_meta.max_pll_vco_hz <= 6000000000ULL) {  
          vfo_max_pll_vco_hz = checked_meta.max_pll_vco_hz;  
        }  
      }  
    }
  }

  // Шаг 2: Выгружаем данные в RAM на основе результатов сканирования
  if (slot_found) {
    uint32_t final_flash_src = 0x10000000 + FLASH_TARGET_OFFSET + (current_active_slot * SLOT_SIZE);
    memcpy(ram_disk_buffer, (const void*)final_flash_src, DISK_SIZE_BYTES);
    
    // Проверяем первые байты имён ВСЕХ трёх файлов в каталоге (Сектор 2)
    uint8_t info_name_byte = ram_disk_buffer[SECTOR_SIZE * 2 + 0];  // Дескриптор INFO
    uint8_t set_name_byte  = ram_disk_buffer[SECTOR_SIZE * 2 + 32]; // Дескриптор SET
    uint8_t log_name_byte  = ram_disk_buffer[SECTOR_SIZE * 2 + 64]; // Дескриптор LOG
    
    // Если хотя бы один файл был стёрт операционной системой ПК
    if (info_name_byte == 0x00 || info_name_byte == 0xE5 || 
        set_name_byte  == 0x00 || set_name_byte  == 0xE5 || 
        log_name_byte  == 0x00 || log_name_byte  == 0xE5) {
        
      Serial.println(F("[Система] Обнаружена пропажа файлов! Принудительное восстановление..."));
      create_default_fat_with_info_file();
      my_FAT = "Структура файлов была нарушена или удалена ПК. Всё восстановлено.";
    } else {
      my_FAT = "Файловая система флэш - корректна. Слот " + String(current_active_slot) + " (seq=" + String(current_max_seq) + ")";
    }
  } else {
    create_default_fat_with_info_file();
    my_FAT = "Файловая система не найдена во всех слотах. Восстановлен дефолт.";
  }

  read_and_parse_INFO_txt();  // Парсер настроек INFO.TXT
  read_and_parse_SET_txt();   // Парсер инженерных настроек SET.TXT
  Serial.flush();
}





/**  
 * @brief Отложенная обработка изменений файлов, записанных с ПК (вызов из loop()).  
 *  
 * Завершает транзакцию записи хоста по USB-MSC: колбэки @ref msc_write_cb  
 * и @ref msc_flush_cb работают в контексте прерывания TinyUSB и лишь  
 * взводят флаги (@c pc_activity_detected, @c pc_written_regions,  
 * @c pc_file_written), а эта функция — единственное безопасное место  
 * для «тяжёлой» реакции: перепарсинга конфигурации и записи во Flash.  
 *  
 * Логика:  
 * - каждый вызов подпитывает сторожевой таймер (`watchdog_update()`);  
 * - реакция срабатывает только после ~1.5 с тишины с момента последней  
 *   записи/flush (@c last_msc_write_time) — защита от разбора  
 *   недописанного файла, пока хост ещё льёт секторы;  
 * - по битовой карте @c pc_written_regions определяется, какие файлы  
 *   затронуты:  
 *   - бит 1 (0x02, INFO.TXT) → @ref read_and_parse_INFO_txt +  
 *     @ref ensure_file_timestamps;  
 *   - бит 2 (0x04, SET.TXT) → @ref read_and_parse_SET_txt +  
 *     @ref ensure_file_timestamps;  
 *   - бит 0 (0x01, бут/FAT/каталог) и бит 3 (0x08, LOG.TXT) —  
 *     учитываются только как факт активности; LOG намеренно не парсится;  
 * - если менялись INFO или SET — образ диска один раз сбрасывается во  
 *   Flash через @ref save_ram_to_flash (даже при правке обоих файлов);  
 * - в конце сбрасываются все флаги: @c pc_activity_detected,  
 *   @c pc_written_regions и @c pc_file_written — последний разблокирует  
 *   передачу, остановленную экстренным флагом из @ref msc_write_cb.  
 *  
 * @note Вызывается каждую итерацию `loop()` на Core 0. Функция не  
 *       «передёргивает» том — remount Windows-сессии выполняется только  
 *       при консольном редактировании (@ref update_info_config_from_console);  
 *       при записи с ПК хост сам видит свои изменения.  
 * @note Если активность ПК продолжается (таймаут не истёк), функция —  
 *       no-op, кроме подпитки watchdog.  
 *  
 * @see msc_write_cb(), msc_flush_cb(), save_ram_to_flash(),  
 *      read_and_parse_INFO_txt(), read_and_parse_SET_txt(),  
 *      ensure_file_timestamps(), init_file_manager()  
 */
void check_and_handle_pc_changes() {
  watchdog_update();    // обновить сторожевой таймер
  if (pc_activity_detected && (millis() - last_msc_write_time > 1500)) {
    // переписываем флэш только один раз, даже если правились оба файла 
    bool config_changed = (pc_written_regions & 0x02) || (pc_written_regions & 0x04);
    if (pc_written_regions & 0x02) {
      Serial.println(F("[Система] Обнаружена корректировка файла: INFO.TXT "));
      read_and_parse_INFO_txt();    // Парсер настроек INFO.TXT
      ensure_file_timestamps();    // проверяет метки даты/времени на файлах
    }  
    if (pc_written_regions & 0x04) {  
      Serial.println(F("[Система] Обнаружена корректировка файла: SET.TXT "));
      read_and_parse_SET_txt();   // Парсер инженерных настроек SET.TXT
      ensure_file_timestamps();    // проверяет метки даты/времени на файлах
    }  
    if (config_changed) {  
      save_ram_to_flash();   // один раз, даже если правились оба файла  
    }
    pc_activity_detected = false; // сброс флага факта записи на диск
    pc_written_regions = 0;       // сброс указателя места записи
    pc_file_written = false;      // сброс флага изменения файлов INFO или SET (true останавливает передачу)
  }
}


// Универсальное редактирование любого параметра в файле INFO.txt из консоли
void update_info_config_from_console(String marker, String new_value) {
  uint32_t root_offset = SECTOR_SIZE * 2;
  uint32_t data_offset = SECTOR_SIZE * 5;

  if (new_value.length() > 32) {
    new_value = new_value.substring(0, 32);
  }

  // Гибкий двухэтапный поиск маркера с учетом пробелов выравнивания
  String search_tag = "[" + marker;
  search_tag.trim(); // Отрезаем случайные пробелы по краям, получили например "[FREQ_CW"

  uint8_t* data_ptr = &ram_disk_buffer[data_offset];
  
  // ЭТАП 1: Ищем само имя маркера в секторе данных
  uint8_t* marker_ptr = (uint8_t*)strstr((const char*)data_ptr, search_tag.c_str());
  uint8_t* target_tag = nullptr;

  if (marker_ptr != nullptr) {
    // ЭТАП 2: От найденного имени ищем ближайший знак закрытия скобки и равенства "]="
    // Просматриваем вперед не более 20 символов ради безопасности
    for (int k = 0; k < 20; k++) {
      if (marker_ptr[k] == ']' && marker_ptr[k + 1] == '=') {
        target_tag = marker_ptr; // Маркер успешно локализован!
        break;
      }
      if (marker_ptr[k] == '\n' || marker_ptr[k] == '\r' || marker_ptr[k] == '\0') {
        break; // Защита: вышли за пределы строки
      }
    }
  }
  
  if (target_tag != nullptr) {
    // позиция записи вычисляется динамически — встаем строго за знак '='
    uint8_t* write_ptr = (uint8_t*)strchr((const char*)target_tag, '=') + 1; 
    
    uint8_t* end_of_old_line = (uint8_t*)strpbrk((const char*)write_ptr, "\r\n");
    
    if (end_of_old_line != nullptr) {
      uint32_t old_val_len = end_of_old_line - write_ptr;
      uint32_t new_val_len = new_value.length();
      
      // БЕЗОПАСНЫЙ РАСЧЕТ: Вычисляем позицию внутри RAM-диска
      uint32_t current_write_pos = end_of_old_line - ram_disk_buffer;
      
      // ИСПРАВЛЕНО: Хвост ограничивается концом кластера INFO.TXT (10 Кб),
      // а не концом всего ОЗУ-диска. Сектор 25 (SET.TXT) теперь под полной защитой!
      uint32_t tail_len = (data_offset + INFO_MAX_BYTES) - current_write_pos;

      // Если длины старого и нового значений не совпадают — раздвигаем или сдвигаем память
      if (new_val_len != old_val_len) {
        uint8_t* new_tail_pos = write_ptr + new_val_len;
        
        // Защита: проверяем, чтобы сдвиг не вылез за физические границы, выделенные под INFO.TXT
        if ((new_tail_pos - ram_disk_buffer) + tail_len <= (data_offset + INFO_MAX_BYTES)) {
          memmove(new_tail_pos, end_of_old_line, tail_len);
        } else {
          Serial.println(F("[Ошибка] Критическое переполнение кластера INFO.TXT! Отмена операции."));
          return;
        }
      }
      
      // Вписываем новое значение параметра
      memcpy(write_ptr, new_value.c_str(), new_val_len);
      
      // Пересчитываем точный размер текстового файла для корневого каталога FAT12
      uint32_t total_file_size = strlen((const char*)data_ptr);
      ram_disk_buffer[root_offset + 28] = (uint8_t)(total_file_size & 0xFF);
      ram_disk_buffer[root_offset + 29] = (uint8_t)((total_file_size >> 8) & 0xFF);
      
      // Сохраняем образ диска во Flash-память RP2040 и обновляем переменные в ОЗУ
      set_dir_timestamp(0, false);   // INFO.TXT — только обновление времени модификации
      save_ram_to_flash();
      read_and_parse_INFO_txt();
      
      // Принудительно перезапускаем сессию для Windows, чтобы обновить файлы в Проводнике
      remount_usb_disk();
      
      Serial.print(F("[Система] Изменение успешно записано! [")); Serial.print(marker); 
      Serial.print(F("] = [")); Serial.print(new_value); Serial.println(F("]"));
    }
  } else {
    Serial.print(F("[Ошибка] Маркер [")); Serial.print(marker); Serial.println(F("] не найден."));
  }
}



// -------------------------------------------------------------------------
// Функция очистки (стирания) файла LOG.TXT
// -------------------------------------------------------------------------
void log_file_clear() {
  uint32_t root_offset = SECTOR_SIZE * 2;         
  uint32_t log_entry_offset = root_offset + 64;   
  uint32_t log_data_offset = SECTOR_SIZE * (5 + INFO_CLUSTERS + SET_CLUSTERS); // ИСПРАВЛЕНО: Сектор 45

  memset(&ram_disk_buffer[log_data_offset], 0, LOG_MAX_BYTES);

  String header = "           === ЖУРНАЛ РАБОТЫ МАЯКА ===\r\n        Версия ПО v." + BCN_VER + " от " + BCN_DAT + "\r\n";
  uint32_t header_len = header.length();
  memcpy(&ram_disk_buffer[log_data_offset], header.c_str(), header_len);

  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(header_len & 0xFF);
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((header_len >> 8) & 0xFF);
  ram_disk_buffer[log_entry_offset + 30] = 0x00;
  ram_disk_buffer[log_entry_offset + 31] = 0x00;

  set_dir_timestamp(2, false);

  save_ram_to_flash();
  Serial.println("[Журнал]  Файл LOG.TXT успешно очищен.");
  // Сообщаем ОС, что накопитель переподключен
  remount_usb_disk();
}






/**  
 * @brief Дозапись одной строки с меткой времени в конец LOG.TXT.  
 *  
 * Формирует строку журнала вида "ДД.ММ.ГГГГ ЧЧ:ММ:СС <сообщение>\r\n":  
 * текущие дата и время берутся из RTC через @ref get_current_date() и  
 * @ref get_current_time(), к ним присоединяется текст @p message.  
 * Строка дописывается в хвост области данных LOG.TXT на RAM-диске  
 * (начало данных — сектор 5 + INFO_CLUSTERS + SET_CLUSTERS = 45,  
 * максимум @c LOG_MAX_BYTES = 100 КБ).  
 *  
 * Логика:  
 * -# читает текущий размер файла из записи корневого каталога  
 *    (сектор 2, смещение 64, байты 28-31, little-endian);  
 * -# если строка не помещается — автоматическая очистка журнала  
 *    через @ref log_file_clear() и перечитывание размера (оставленный  
 *    заголовок учитывается);  
 * -# копирует строку в хвост файла и обновляет 4-байтовое поле размера  
 *    в записи каталога;  
 * -# обновляет метку времени записи (@ref set_dir_timestamp, индекс 2);  
 * -# сбрасывает весь образ во Flash (@ref save_ram_to_flash) и  
 *    перемонтирует USB-том для ПК (@ref remount_usb_disk).  
 *  
 * @param[in] message  Текст события без завершающего перевода строки  
 *                     (префикс даты/времени и "\r\n" добавляются внутри).  
 *  
 * @warning Каждый вызов стоит одно стирание+запись Flash-слота и один  
 *          цикл remount USB (~1.5 с паузы). Для пакетной записи  
 *          нескольких строк используйте @ref log_file_write_block —  
 *          там сохранение и перемонтирование выполняются один раз на  
 *          весь блок, что критично для ресурса Flash.  
 * @note Используется для одиночных событий: команды оператора  
 *       (RESET, remount, format disk), завершение сеансов передачи  
 *       (под @c debug_flag), очистка журнала и т.п.  
 *  
 * @see log_file_write_block(), log_file_clear(), save_ram_to_flash(),  
 *      remount_usb_disk(), set_dir_timestamp()  
 */
void log_file_write_line(String message) {
  uint32_t root_offset = SECTOR_SIZE * 2;
  uint32_t log_entry_offset = root_offset + 64;
  uint32_t log_data_offset = SECTOR_SIZE * (5 + INFO_CLUSTERS + SET_CLUSTERS);

  uint32_t current_size = ram_disk_buffer[log_entry_offset + 28] | 
                         (ram_disk_buffer[log_entry_offset + 29] << 8) |
                         (ram_disk_buffer[log_entry_offset + 30] << 16) |
                         (ram_disk_buffer[log_entry_offset + 31] << 24);
  // формат строки
  // дата время сообщение
  String formatted_msg = get_current_date() + " " + get_current_time() + " " + message + "\r\n";
  uint32_t msg_len = formatted_msg.length();

  if (current_size + msg_len >= (LOG_MAX_BYTES - 1)) {
    Serial.println("[Журнал]  Предупреждение: Лог 100 Кб заполнен! Автоматическая очистка...");
    log_file_clear(); 
    current_size = ram_disk_buffer[log_entry_offset + 28] | (ram_disk_buffer[log_entry_offset + 29] << 8);
  }

  uint8_t* write_pointer = &ram_disk_buffer[log_data_offset + current_size];
  memcpy(write_pointer, formatted_msg.c_str(), msg_len);

  uint32_t new_size = current_size + msg_len;
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(new_size & 0xFF);
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((new_size >> 8) & 0xFF);
  ram_disk_buffer[log_entry_offset + 30] = (uint8_t)((new_size >> 16) & 0xFF);
  ram_disk_buffer[log_entry_offset + 31] = (uint8_t)((new_size >> 24) & 0xFF);

  set_dir_timestamp(2,false);
  save_ram_to_flash();
  Serial.print("[Журнал]  Строка добавлена. Объем лога: "); Serial.print(new_size); Serial.println(" байт.");

  // Сообщаем ОС, что накопитель переподключен
  remount_usb_disk();
}









/**  
 * @brief Сборка и запись актуального содержимого SET.TXT в область данных RAM-диска.  
 *  
 * Формирует файл инженерных настроек заново из текущих глобальных  
 * переменных железа и записывает его в сектор 25 RAM-диска  
 * (@c ram_disk_buffer). Структура документа фиксированная, значения  
 * подставляются из рантайм-переменных:  
 * - пины: `PIN_FREQ_OUT`, `PIN_AMP_ACT`, `PIN_SUBBAND_0..3`,  
 *   `PIN_PWR_SI/DS/BM/DL` — через лямбду @c pin_to_str, превращающую  
 *   -1 в строку "NC";  
 * - `SI5351_CLK_OUT` — активный выходной канал Si5351;  
 * - `SCAN_EXCLUDE` — список исключённых из сканирования пинов I2C;  
 * - `DEBUG`, `LED_ENABLE` — служебные флаги;  
 * - `CLIM_SAMPLE_MIN`, `CLIM_REPORT_MIN` — интервалы климат-мониторинга;  
 * - `FLASH_SLOT`, `FLASH_SEQ` — статистика износа Flash  
 *   (активный слот и счётчик версий);  
 * - `[SCAN_RESULT]` — блок результатов I2C-сканирования из параметра  
 *   @p scan_results.  
 *  
 * После сборки строки файл обрезается до `SET_MAX_BYTES - 10` с  
 * добавлением маркера `[EOF]`, область данных зануляется  
 * (`memset`), содержимое копируется в RAM-диск, а размер файла  
 * обновляется в записи корневого каталога (смещение 32, байты 28-31).  
 *  
 * Завершает работу обновлением метки времени записи SET.TXT  
 * (@ref set_dir_timestamp, индекс 1), сбросом образа во Flash  
 * (@ref save_ram_to_flash) и принудительным перемонтированием тома  
 * на ПК (@ref remount_usb_disk), чтобы хост перечитал изменённый файл.  
 *  
 * @param[in] scan_results  Многострочная строка результатов I2C-  
 *                          сканирования (формируется в @ref I2C_Scanner  
 *                          через @ref fmt_dev). Также сохраняется в  
 *                          глобальную @c scan_result_data.  
 *  
 * @note Функция — зеркало генератора SET.TXT внутри  
 *       @ref create_default_fat_with_info_file: обе собирают один и тот  
 *       же формат, поэтому изменения шаблона нужно вносить в два места.  
 * @warning Вызывает @ref save_ram_to_flash и @ref remount_usb_disk —  
 *          недёшёвый вызов (стирание Flash-слота + переподключение  
 *          USB-MSC); не вызывать в горячем цикле.  
 *  
 * @see I2C_Scanner(), read_and_parse_SET_txt(), create_default_fat_with_info_file(),  
 *      save_ram_to_flash(), remount_usb_disk()  
 */
void save_hardware_settings_to_file(String scan_results) {
  uint32_t root_offset = SECTOR_SIZE * 2;
  uint32_t set_entry_offset = root_offset + 32; 
  uint32_t set_data_offset = SECTOR_SIZE * 25; // Сектор 25

  auto pin_to_str = [](int p) -> String {
    return (p == -1) ? "NC" : String(p);
  };

  // Сохраняем переданные результаты сканирования в глобальный буфер
  scan_result_data = scan_results; 

  String content = "";
  content.reserve(512);
  content += "=== ИНЖЕНЕРНЫЕ НАСТРОЙКИ МАЯКА ===\r\n";
  content += "[PIN_FREQ_OUT ]=" + pin_to_str(pin_freq_out) + "\r\n";
  content += "[PIN_AMP_ACT  ]=" + pin_to_str(pin_amp_act) + "\r\n\r\n";
  content += "// Выходной канал частоты передачи Si5351: 0 = CLK0, 1 = CLK1, 2 = CLK2\r\n";
  content += "[SI5351_CLK_OUT]=" + String(si5351_clk_tx_out) + "\r\n\r\n";
  content += "// Пины кода поддиапазона (4 пина)\r\n";
  content += "[PIN_SUBBAND_0]=" + pin_to_str(subband_pins[0]) + "\r\n";
  content += "[PIN_SUBBAND_1]=" + pin_to_str(subband_pins[1]) + "\r\n";
  content += "[PIN_SUBBAND_2]=" + pin_to_str(subband_pins[2]) + "\r\n";
  content += "[PIN_SUBBAND_3]=" + pin_to_str(subband_pins[3]) + "\r\n\r\n";
  content += "// Пины питания шины (NC если не назначены)\r\n";
  content += "[PIN_PWR_SI   ]=" + pin_to_str(pin_pwr_si) + "\r\n";
  content += "[PIN_PWR_DS   ]=" + pin_to_str(pin_pwr_ds) + "\r\n";
  content += "[PIN_PWR_BM   ]=" + pin_to_str(pin_pwr_bm) + "\r\n";
  content += "[PIN_PWR_DL   ]=" + pin_to_str(pin_pwr_dl) + "\r\n\r\n";
  content += "// Исключения из сканирования шин\r\n";
  content += "[SCAN_EXCLUDE ]=" + scan_exclude_list + "\r\n\r\n";
  content += "// Режим отладки: 1 — служебные сообщения в Serial, 0 — выкл\r\n";
  content += "[DEBUG]=" + String(debug_flag ? 1 : 0) + "\r\n\r\n";
  content += "// Управление светодиодной индикацией: 1 — включена, 0 — выключена\r\n";
  content += "[LED_ENABLE]=" + String(led_enable_flag ? 1 : 0) + "\r\n\r\n";
  // СОХРАНЕНИЕ ТЕКУЩИХ ИНТЕРВАЛОВ ПРИ ПЕРЕЗАПИСИ ИЛИ ИНИЦИАЛИЗАЦИИ ДИСКА
  content += "=== ПЕРИОДИЧНОСТЬ КЛИМАТИЧЕСКОГО МОНИТОРИНГА ===\r\n";
  content += "[CLIM_SAMPLE_MIN]=" + String(climate_sample_interval_min) + "\r\n";
  content += "[CLIM_REPORT_MIN]=" + String(climate_report_interval_min) + "\r\n\r\n";
  content += "=== СТАТИСТИКА ИЗНОСА ФЛЭШ-ПАМЯТИ ===\r\n";
  content += "[FLASH_SLOT   ]=" + String(current_active_slot != -1 ? current_active_slot : 0) + "\r\n";
  content += "[FLASH_SEQ    ]=" + String(current_max_seq != 0 ? current_max_seq : 1) + "\r\n\r\n";
  content += "=== УСТРОЙСТВА НА ШИНЕ I2C ===\r\n";
  content += "[SCAN_RESULT]\r\n" + scan_results + "\r\n\r\n"; 
  content += "[EOF]";

  uint32_t total_len = content.length();
  if (total_len >= SET_MAX_BYTES) {
    content = content.substring(0, SET_MAX_BYTES - 10) + "\r\n[EOF]";
    total_len = content.length();
  }

  memset(&ram_disk_buffer[set_data_offset], 0, SET_MAX_BYTES);
  memcpy(&ram_disk_buffer[set_data_offset], content.c_str(), total_len);

  ram_disk_buffer[set_entry_offset + 28] = (uint8_t)(total_len & 0xFF);
  ram_disk_buffer[set_entry_offset + 29] = (uint8_t)((total_len >> 8) & 0xFF);
  ram_disk_buffer[set_entry_offset + 30] = 0x00;
  ram_disk_buffer[set_entry_offset + 31] = 0x00;

  set_dir_timestamp(1, false);   // SET.TXT
  save_ram_to_flash();
  remount_usb_disk(); 
}




// Проверка, входит ли конкретный пин в список исключений SCAN_EXCLUDE
// Автоматическое исключение ВСЕХ назначенных в системе пинов + ручного списка
bool is_pin_excluded_from_scan(int pin) {
  // 1. АВТО-ИСКЛЮЧЕНИЕ: Защищаем пин ВЧ-выхода и пин активации УМ
  if (pin == pin_freq_out) return true;
  if (pin == pin_amp_act)  return true;

  // 2. АВТО-ИСКЛЮЧЕНИЕ: Защищаем все 4 пина кода поддиапазона
  for (int k = 0; k < 4; k++) {
    if (pin == subband_pins[k]) return true;
  }

  // 3. АВТО-ИСКЛЮЧЕНИЕ: Защищаем пины питания устройств шины (если они назначены, т.е. не равны -1)
  if (pin_pwr_si != -1 && pin == pin_pwr_si) return true;
  if (pin_pwr_ds != -1 && pin == pin_pwr_ds) return true;
  if (pin_pwr_bm != -1 && pin == pin_pwr_bm) return true;
  if (pin_pwr_dl != -1 && pin == pin_pwr_dl) return true;

  // 4. ТЕКСТОВЫЕ ИСКЛЮЧЕНИЯ: Проверяем ручной список SCAN_EXCLUDE (если там не написано "NC" или пусто)
  if (scan_exclude_list.length() == 0 || scan_exclude_list.equalsIgnoreCase("NC")) return false;
  
  String target = String(pin);
  int start = 0;
  
  while (true) {
    int comma_idx = scan_exclude_list.indexOf(',', start);
    String token = (comma_idx == -1) ? scan_exclude_list.substring(start) : scan_exclude_list.substring(start, comma_idx);
    token.trim();
    
    if (token.equals(target)) return true;
    if (comma_idx == -1) break;
    start = comma_idx + 1;
  }
  return false;
}



// Принудительное форматирование и перезапись всех файлов на настройки по умолчанию
void force_reset_to_default_disk() {
  Serial.println(F("[Система] Запущено полное восстановление диска по умолчанию..."));
  // пины по умолчанию
  // пины шифра поддиапазона
  subband_pins[0] = 6;
  subband_pins[1] = 7;
  subband_pins[2] = 8;
  subband_pins[3] = 9;
  pin_freq_out = 10;  // выход DDS-генератора
  pin_amp_act  = 11;  // выход управления усилителем
  // пины включения питания модулей
  // -1 означает NC (Не назначен / Not Connected)
  pin_pwr_si   = -1;  // генератор SI5351
  pin_pwr_ds   = -1;  // часы RTC
  pin_pwr_bm   = -1;  // климатический датчик
  pin_pwr_dl   = -1;  // дисплей

  // при формате лимиты возвращаются на паспортные:  
  vfo_max_clk_sys_hz = PASSPORT_CLK_SYS_MAX_HZ;  
  vfo_max_pll_vco_hz = PASSPORT_PLL_VCO_MAX_HZ;

  // 1. Генерирует чистую структуру FAT12 в ОЗУ и сама вызывает save_ram_to_flash()
  create_default_fat_with_info_file(); 
  
  // 2. Сразу же обновляем глобальные переменные в ОЗУ из нового дефолтного файла
  read_and_parse_INFO_txt();  
  read_and_parse_SET_txt(); 
  
  // 3. Жестко уведомляем Windows, чтобы он перечитал файловую систему
  remount_usb_disk();
  
  Serial.println(F("[Система] Все файлы успешно перезаписаны на дефолтные!"));
}




/**  
 * @brief Дозапись произвольного многострочного блока в конец LOG.TXT.  
 *  
 * Атомарно (за одну транзакцию) добавляет готовый блок строк @p block  
 * в область данных LOG.TXT на RAM-диске (начало данных — сектор  
 * 5 + INFO_CLUSTERS + SET_CLUSTERS = 45). В отличие от  
 * @ref log_file_write_line, блок не получает префикса «дата время» —  
 * вызывающий код формирует содержимое сам; к концу блока добавляется  
 * завершающий "\r\n".  
 *  
 * Ключевая оптимизация: на весь блок выполняется ОДИН вызов  
 * @ref save_ram_to_flash (стирание + запись Flash-слота с ротацией  
 * износа) и ОДИН цикл @ref remount_usb_disk — в отличие от  
 * построчной записи через log_file_write_line, где каждая строка  
 * обходится стиранием сектора Flash.  
 *  
 * Логика:  
 * -# читает текущий размер файла из записи каталога (сектор 2,  
 *    смещение 64, байты 28-31, little-endian);  
 * -# при переполнении `LOG_MAX_BYTES` (100 КБ) — автоматическая  
 *    очистка через @ref log_file_clear и перечитывание размера  
 *    (оставленный заголовок учитывается);  
 * -# копирует блок в хвост файла, обновляет поле размера в каталоге;  
 * -# обновляет метку времени записи (@ref set_dir_timestamp, индекс 2);  
 * -# сбрасывает образ во Flash и перемонтирует USB-том для ПК.  
 *  
 * @param[in] block  Многострочная строка для дозаписи (например,  
 *                   климатическая таблица из @ref climate_write_report).  
 *  
 * @note Используется для блочных отчётов — прежде всего таблицы  
 *       климат-мониторинга (@ref climate_write_report в climate_log.cpp).  
 * @warning Если блок не помещается даже в пустой файл после очистки,  
 *          повторной защиты нет — запись выполнится поверх лимита;  
 *          размеры блоков под контролем вызывающего кода.  
 *  
 * @see log_file_write_line(), log_file_clear(), save_ram_to_flash(),  
 *      remount_usb_disk(), climate_write_report()  
 */
void log_file_write_block(const String& block) {  
  uint32_t root_offset      = SECTOR_SIZE * 2;  
  uint32_t log_entry_offset = root_offset + 64;  
  uint32_t log_data_offset  = SECTOR_SIZE * (5 + INFO_CLUSTERS + SET_CLUSTERS);  
  
  uint32_t current_size = ram_disk_buffer[log_entry_offset + 28] |  
                         (ram_disk_buffer[log_entry_offset + 29] << 8) |  
                         (ram_disk_buffer[log_entry_offset + 30] << 16) |  
                         (ram_disk_buffer[log_entry_offset + 31] << 24);  
  
  String formatted = block + "\r\n";   // завершаем последнюю строку блока  
  uint32_t msg_len = formatted.length();  
  
  if (current_size + msg_len >= (LOG_MAX_BYTES - 1)) {  
    Serial.println("[Журнал]  Лог заполнен! Автоочистка...");  
    log_file_clear();   // если внутри есть save+remount — они выполнятся лишний раз  
    // перечитываем размер — clear оставляет заголовок длиной header_len  
    current_size = ram_disk_buffer[log_entry_offset + 28] |  
                  (ram_disk_buffer[log_entry_offset + 29] << 8) |  
                  (ram_disk_buffer[log_entry_offset + 30] << 16) |  
                  (ram_disk_buffer[log_entry_offset + 31] << 24);
  }

  memcpy(&ram_disk_buffer[log_data_offset + current_size], formatted.c_str(), msg_len);  
  
  uint32_t new_size = current_size + msg_len;  
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(new_size);  
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)(new_size >> 8);  
  ram_disk_buffer[log_entry_offset + 30] = (uint8_t)(new_size >> 16);  
  ram_disk_buffer[log_entry_offset + 31] = (uint8_t)(new_size >> 24);  
  
  set_dir_timestamp(2, false);

  save_ram_to_flash();              // <-- один раз на весь блок  
  Serial.print("[Журнал]  Блок добавлен, объем: ");  
  Serial.println(new_size);  
  
  remount_usb_disk();  
}


// перемонтирование USB диска
void remount_usb_disk() {
  usb_msc.setUnitReady(false);      // перемонтирование
    uint32_t t0 = millis();  
    while (millis() - t0 < 1500) {  // пауза с кормлением watchdog 
      watchdog_update();  
      delay(50);  
    } 
  usb_msc.setUnitReady(true); 
}



// Проверка меток даты/времени файлов: если поля создания нулевые — проставить текущие  
void ensure_file_timestamps() {
  if (rtc_year < 2020) return;   // RTC ещё не готов — отложить до следующего вызова
  uint32_t root_offset = SECTOR_SIZE * 2;  
  const char* names[3] = { "INFO.TXT", "SET.TXT", "LOG.TXT" };  
  bool changed = false;  
  
  for (uint8_t idx = 0; idx < 3; idx++) {  
    uint32_t e = root_offset + (uint32_t)idx * 32;  
  
    // Поля создания: 14-15 (время), 16-17 (дата). Проверяем дату (16-17) — главный признак  
    uint16_t create_date = ram_disk_buffer[e + 16] | (ram_disk_buffer[e + 17] << 8);  
    uint16_t write_date  = ram_disk_buffer[e + 24] | (ram_disk_buffer[e + 25] << 8);  
  
    if (create_date == 0 || write_date == 0) {  
      // with_create = true: заполнит и creation, и write time/date  
      set_dir_timestamp(idx, true);  
      Serial.printf("[Диск] %s: проставлены отсутствующие метки даты/времени\r\n", names[idx]);  
      changed = true;  
    }  
  }  
  
  if (changed) {  
    save_ram_to_flash();   // зафиксировать метки во Flash, иначе пропадут при выключении  
  }  
}
