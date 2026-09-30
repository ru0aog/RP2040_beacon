/**  
 * ============================================================================  
 *  file_manager.cpp — USB-накопитель (MSC) в ОЗУ + конфиг маяка INFO.TXT  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Представляет RP2040 для ПК как обычную USB-флешку (Mass Storage Class),  
 *  на которой лежит единственный текстовый файл INFO.TXT со всеми настройками  
 *  маяка (позывной, локатор, текст, расписание, частоты, скорости). Отредактировав  
 *  этот файл в блокноте, оператор меняет параметры без перепрошивки.  
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

// Физическое выделение памяти под инженерные переменные железа
// Значения по умолчанию, если теги не найдены
uint32_t CW_DOT_TIME_MS  = 60;              // Время точки в мс (по умолчанию ~20 WPM)
volatile uint32_t RTTY_BIT_TIME_US = 22000; // Время одного бита RTTY в мкс (по умолчанию 45.45 Бод)

int subband_pins[4] = {6, 7, 8, 9};  // пины шифра поддиапазона
int pin_freq_out = 10;  // выход DDS-генератора
int pin_amp_act  = 11;  // выход управления усилителем
// пины включения питания модулей
// -1 означает NC (Не назначен / Not Connected)
int pin_pwr_si   = -1;  // генератор SI5351
int pin_pwr_ds   = -1;  // часы RTC
int pin_pwr_bm   = -1;  // климатический датчик
int pin_pwr_dl   = -1;  // дисплей


// Выделение ОЗУ под таблицу расписания задач
TaskItem beacon_schedule[MAX_SCHEDULE_TASKS];

// Переменные мониторинга текущего состояния флэша (Wear Leveling)
int32_t  current_active_slot = -1; // активный слот
uint32_t current_max_seq     = 0;  // число перезаписей

volatile bool pc_file_written = false;



// Выделение памяти под буфер диска в ОЗУ
alignas(4) static uint8_t ram_disk_buffer[DISK_SIZE_BYTES];

// Переменные времени для отслеживания ПК
volatile uint32_t last_msc_write_time = 0;
bool pc_activity_detected = false;

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

// msc_write_cb - единственное место, где ПК действительно пишет сектор
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {  
  if (lba >= SECTOR_COUNT) return -1;  
  memcpy(&ram_disk_buffer[lba * SECTOR_SIZE], buffer, bufsize);  
  last_msc_write_time = millis();  
  pc_activity_detected = true;  
  // Прерываем эфир только при изменении конфигурации:  
  // сектора 0-4 (Boot/FAT/каталог), INFO.TXT (5-24), SET.TXT (25-44)  
  uint32_t last_lba = lba + (bufsize / SECTOR_SIZE) - 1;  
  if (lba <= 44) {  // покрывает и случай last_lba > 44 при захвате сектора 44  
    pc_file_written = true;  // Выставляем флаг для экстренного останова передачи
  }  
  return bufsize;  
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

  // Выполняем физическую безопасную запись с отключением прерываний на Core 0
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(target_flash_addr, SLOT_SIZE);
  flash_range_program(target_flash_addr, ram_disk_buffer, DISK_SIZE_BYTES);
  restore_interrupts(ints);
  flash_flush_cache();

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

// Генерация диска FAT12 с 48 записями каталога и сдвигом данных на Сектор 5
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
    "[TASK_01]=0,15:15,3601500,CW\r\n"
    "[TASK_02]=0,15:18,3601585,RTTY\r\n"
    "[TASK_03]=0,15:20,3601307,IFKP\r\n"
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
  String dynamic_set_content = "";
  dynamic_set_content.reserve(512);
  dynamic_set_content += "=== ИНЖЕНЕРНЫЕ НАСТРОЙКИ МАЯКА ===\r\n";
  dynamic_set_content += "[PIN_FREQ_OUT ]=" + pin_to_str(pin_freq_out) + "\r\n";
  dynamic_set_content += "[PIN_AMP_ACT  ]=" + pin_to_str(pin_amp_act) + "\r\n\r\n";
  dynamic_set_content += "// Пины кода поддиапазона (4 пина)\r\n";
  dynamic_set_content += "[SUBBAND_PIN_0]=" + pin_to_str(subband_pins[0]) + "\r\n";
  dynamic_set_content += "[SUBBAND_PIN_1]=" + pin_to_str(subband_pins[1]) + "\r\n";
  dynamic_set_content += "[SUBBAND_PIN_2]=" + pin_to_str(subband_pins[2]) + "\r\n";
  dynamic_set_content += "[SUBBAND_PIN_3]=" + pin_to_str(subband_pins[3]) + "\r\n\r\n";
  dynamic_set_content += "// Пины питания шины (NC если не назначены)\r\n";
  dynamic_set_content += "[BUS_PWR_SI   ]=" + pin_to_str(pin_pwr_si) + "\r\n";
  dynamic_set_content += "[BUS_PWR_DS   ]=" + pin_to_str(pin_pwr_ds) + "\r\n";
  dynamic_set_content += "[BUS_PWR_BM   ]=" + pin_to_str(pin_pwr_bm) + "\r\n";
  dynamic_set_content += "[BUS_PWR_DL   ]=" + pin_to_str(pin_pwr_dl) + "\r\n\r\n";
  dynamic_set_content += "// Дополнительные исключения из сканирования шин\r\n";
  dynamic_set_content += "[SCAN_EXCLUDE ]=16,23,24,25\r\n\r\n";
  dynamic_set_content += "=== СТАТИСТИКА ИЗНОСА ФЛЭШ-ПАМЯТИ ===\r\n";
  dynamic_set_content += "[FLASH_SLOT   ]=" + String(current_active_slot != -1 ? current_active_slot : 0) + "\r\n";
  dynamic_set_content += "[FLASH_SEQ    ]=" + String(current_max_seq != 0 ? current_max_seq : 1) + "\r\n\r\n";
  dynamic_set_content += "=== УСТРОЙСТВА НА ШИНЕ I2C ===\r\n";
  dynamic_set_content += "[SCAN_RESULT]\r\nСканирование не проводилось.\r\n\r\n";
  dynamic_set_content += "[EOF]";
  
  uint32_t set_len = dynamic_set_content.length();
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
  memcpy(&ram_disk_buffer[set_data_offset], dynamic_set_content.c_str(), set_len);

  // Сектор 45: Данные LOG.TXT (5 + 20 + 20)
  uint32_t log_data_offset = SECTOR_SIZE * 45; // Сектор 45 (5 + 20 + 20)
  memcpy(&ram_disk_buffer[log_data_offset], default_log_content.c_str(), log_len);

  save_ram_to_flash();
  Serial.println("[Система] Структура диска обновлена: INFO (10Кб), SET (10Кб) и LOG (100Кб) готовы на Секторе 5!");
}






// Внутренняя функция побайтового разбора маркеров
// ПРАВКА: Полный парсер INFO.TXT, изолированный на Секторах 5..24 со сбором всех переменных и расписания
void read_file_to_variable() {
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
    if (ram_disk_buffer[i] == '[' && (i == scan_start || ram_disk_buffer[i-1] == '\n' || ram_disk_buffer[i-1] == '\r')) {
      int32_t start_idx = -1;
      String* target_str = nullptr;
      bool is_task_line = false;

      // --- БЛОК А: Сборка одиночных текстовых и частотных маркеров ---
      if (ram_disk_buffer[i+1] == 'C' && ram_disk_buffer[i+2] == 'A' && ram_disk_buffer[i+3] == 'L' && ram_disk_buffer[i+4] == 'L' && ram_disk_buffer[i+5] == ']') {
        start_idx = i + 6; target_str = &my_call_variable;
      }
      else if (ram_disk_buffer[i+1] == 'Q' && ram_disk_buffer[i+2] == 'T' && ram_disk_buffer[i+3] == 'H' && ram_disk_buffer[i+4] == ']') {
        start_idx = i + 5; target_str = &my_qth_variable;
      }
      else if (ram_disk_buffer[i+1] == 'T' && ram_disk_buffer[i+2] == 'E' && ram_disk_buffer[i+3] == 'X' && ram_disk_buffer[i+4] == 'T' && ram_disk_buffer[i+5] == ']') {
        start_idx = i + 6; target_str = &my_text_variable;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "CW_WPM    ]", 11) == 0) {
        start_idx = i + 12; target_str = &my_cw_wpm_var;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "RTTY_SPEED]", 11) == 0) {
        start_idx = i + 12; target_str = &my_rtty_baud_var;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "FREQ_CW   ]", 11) == 0) {
        start_idx = i + 12; target_str = &my_freq_cw_var;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "RTTY_MARK ]", 11) == 0) {
        start_idx = i + 12; target_str = &my_rtty_mark_var;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "RTTY_SHIFT]", 11) == 0) {
        start_idx = i + 12; target_str = &my_rtty_shift_var;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "RTTY_INVERT]", 12) == 0) {
        start_idx = i + 13; target_str = &my_rtty_invert_var;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "FREQ_IFKP ]", 11) == 0) {
        start_idx = i + 12; target_str = &my_freq_ifkp_var;
      }
      // --- БЛОК Б: Определение новой матрицы расписания [TASK_XX] ---
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "TASK_", 5) == 0) {
        uint32_t close_bracket = i;
        while (close_bracket < scan_end && ram_disk_buffer[close_bracket] != ']') close_bracket++;
        if (ram_disk_buffer[close_bracket] == ']') {
          start_idx = close_bracket + 1;
          is_task_line = true;
        }
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
  Serial.print("Частота  [FREQ_IFKP ]: "); Serial.print(my_freq_ifkp_var); Serial.println(" Hz");

  // Вывод аппаратной конфигурации пинов из SET.TXT
  Serial.println("--- Аппаратная конфигурация (SET.TXT) ---");
  Serial.print("Пин ВЧ-выхода   (FREQ_OUT): "); Serial.println(pin_freq_out);
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
// 
void init_flash_disk() {
  // Сигнализируем Windows, что медианоситель успешно вставлен и готов к работе
  usb_msc.setUnitReady(true);
  
                
}


// Считывание данных в ОЗУ из Flash
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

  read_file_to_variable();  // Парсер INFO.TXT
  read_hardware_settings(); // Парсер инженерных настроек SET.TXT
  Serial.flush();

}



// Функция проверки изменений от ПК для loop()
void check_and_handle_pc_changes() {
  if (pc_activity_detected && (millis() - last_msc_write_time > 1500)) {
    Serial.println("> ОБНАРУЖЕНА КОРРЕКТИРОВКА ФАЙЛА");
    pc_activity_detected = false;
    
    save_ram_to_flash();
    read_file_to_variable();
    read_hardware_settings(); // Перечитываем пины, если оператор изменил SET.TXT

    //usb_msc.setUnitReady(false); 
    //delay(1500);                  
    //usb_msc.setUnitReady(true);
    
    //print_current_settings();
    pc_file_written = false; // Сбрасываем флаг только ПОСЛЕ обновления строк
  }
}


// Функция редактирования любого параметра в файле INFO.txt из консоли
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
    // ИСПРАВЛЕНО: Теперь позиция записи вычисляется динамически — встаем строго за знак '='
    uint8_t* write_ptr = (uint8_t*)strchr((const char*)target_tag, '=') + 1; 
    
    uint8_t* end_of_old_line = (uint8_t*)strpbrk((const char*)write_ptr, "\r\n");
    
    if (end_of_old_line != nullptr) {
      uint32_t old_val_len = end_of_old_line - write_ptr;
      uint32_t new_val_len = new_value.length();
      
      // БЕЗОПАСНЫЙ РАСЧЕТ: Вычисляем длину хвоста диска до самого конца выделенного сектора
      uint32_t current_write_pos = end_of_old_line - ram_disk_buffer;
      uint32_t tail_len = DISK_SIZE_BYTES - current_write_pos;

      // Если длины старого и нового значений не совпадают — раздвигаем или сдвигаем память
      if (new_val_len != old_val_len) {
        uint8_t* new_tail_pos = write_ptr + new_val_len;
        
        // Защита: проверяем, чтобы сдвиг не вылез за физические границы ОЗУ-диска
        if ((new_tail_pos - ram_disk_buffer) + tail_len < DISK_SIZE_BYTES) {
          memmove(new_tail_pos, end_of_old_line, tail_len);
        }
      }
      
      // Вписываем новое значение параметра
      memcpy(write_ptr, new_value.c_str(), new_val_len);
      
      // Пересчитываем точный размер текстового файла для корневого каталога FAT12
      uint32_t total_file_size = strlen((const char*)data_ptr);
      ram_disk_buffer[root_offset + 28] = (uint8_t)(total_file_size & 0xFF);
      ram_disk_buffer[root_offset + 29] = (uint8_t)((total_file_size >> 8) & 0xFF);
      
      // Сохраняем образ диска во Flash-память RP2040 и обновляем переменные в ОЗУ
      save_ram_to_flash();
      read_file_to_variable();
      
      // Принудительно перезапускаем сессию для Windows
      usb_msc.setUnitReady(false); // Сообщаем ОС, что накопитель извлечен
      // Даем операционной системе ПК ровно 1.5 секунды, чтобы она гарантированно 
      // закрыла файл в Блокноте, удалила кэш секторов и поняла, что флешку вынули!
      delay(1500);                 
      usb_msc.setUnitReady(true);  // Сообщаем Windows, что вставлен новый исправный диск
      
      Serial.print("[Система] Изменение успешно записано! ["); Serial.print(marker); 
      Serial.print("] = ["); Serial.print(new_value); Serial.println("]");
    }
  } else {
    Serial.print("[Ошибка] Маркер ["); Serial.print(marker); Serial.println("] не найден.");
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

  save_ram_to_flash();
  Serial.println("[Журнал]  Файл LOG.TXT успешно очищен.");
  // Сообщаем ОС, что накопитель переподключен
  usb_msc.setUnitReady(false); 
  delay(1500); 
  usb_msc.setUnitReady(true);
}

// -------------------------------------------------------------------------
// Функция дозаписи текстовой строки в конец файла LOG.TXT
// -------------------------------------------------------------------------
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

  save_ram_to_flash();
  Serial.print("[Журнал]  Строка добавлена. Объем лога: "); Serial.print(new_size); Serial.println(" байт.");

  // Сообщаем ОС, что накопитель переподключен
  usb_msc.setUnitReady(false); 
  delay(1500); 
  usb_msc.setUnitReady(true); 
}



// Вспомогательный инлайн для обработки значений пинов (парсит числа или возвращает -1 для NC)
static inline int parse_pin_value(const String& val) {
  String tmp = val;
  tmp.trim();
  if (tmp.equalsIgnoreCase("NC") || tmp.length() == 0) return -1;
  return tmp.toInt();
}

// Функция побайтового разбора маркеров файла SET.TXT
// Парсер инженерных настроек с защитой от сброса в 0
void read_hardware_settings() {
  flash_flush_cache();
  
  String s_freq_out = "", s_amp_act = "";
  String s_subband[4] = {"", "", "", ""};
  String s_pwr_si = "", s_pwr_ds = "", s_pwr_bm = "", s_pwr_dl = "";
  scan_exclude_list = "";
  scan_result_data  = "";

  uint32_t scan_start = 25 * SECTOR_SIZE; // Сектор 25
  uint32_t scan_end   = scan_start + (SET_CLUSTERS * SECTOR_SIZE);

  for (uint32_t i = scan_start; i < scan_end - 15; i++) {
    if (ram_disk_buffer[i] == '[') {
      int32_t start_idx = -1;
      String* target_str = nullptr;

      // Динамически ищем закрывающую скобку ']', чтобы пробелы выравнивания не ломали strncmp
      uint32_t close_bracket_idx = 0;
      for (uint32_t k = i; k < i + 20; k++) {
        if (ram_disk_buffer[k] == ']') {
          close_bracket_idx = k;
          break;
        }
      }

      if (close_bracket_idx == 0) continue; // Битый маркер без скобки

      // Сравниваем чистые имена тегов, игнорируя пробелы внутри скобок
      if (strncmp((const char*)&ram_disk_buffer[i+1], "PIN_FREQ_OUT", 12) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_freq_out;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "PIN_AMP_ACT", 11) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_amp_act;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SUBBAND_PIN_0", 13) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_subband[0];
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SUBBAND_PIN_1", 13) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_subband[1];
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SUBBAND_PIN_2", 13) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_subband[2];
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SUBBAND_PIN_3", 13) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_subband[3];
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "BUS_PWR_SI", 10) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_pwr_si;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "BUS_PWR_DS", 10) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_pwr_ds;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "BUS_PWR_BM", 10) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_pwr_bm;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "BUS_PWR_DL", 10) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &s_pwr_dl;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SCAN_EXCLUDE", 12) == 0) {
        start_idx = close_bracket_idx + 1; target_str = &scan_exclude_list;
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
}



// Автоматическая сборка структуры SET.TXT и запись её в сектор данных RAM-диска
// Сохранение настроек железа на Сектор 25
void save_hardware_settings_to_file(String scan_results) {
  uint32_t root_offset = SECTOR_SIZE * 2;
  uint32_t set_entry_offset = root_offset + 32; 
  uint32_t set_data_offset = SECTOR_SIZE * 25; // ИСПРАВЛЕНО: Сектор 25

  auto pin_to_str = [](int p) -> String {
    return (p == -1) ? "NC" : String(p);
  };

  String content = "";
  content.reserve(512);
  content += "=== ИНЖЕНЕРНЫЕ НАСТРОЙКИ МАЯКА ===\r\n";
  content += "[PIN_FREQ_OUT ]=" + pin_to_str(pin_freq_out) + "\r\n";
  content += "[PIN_AMP_ACT  ]=" + pin_to_str(pin_amp_act) + "\r\n\r\n";
  content += "// Пины кода поддиапазона (4 пина)\r\n";
  content += "[SUBBAND_PIN_0]=" + pin_to_str(subband_pins[0]) + "\r\n";
  content += "[SUBBAND_PIN_1]=" + pin_to_str(subband_pins[1]) + "\r\n";
  content += "[SUBBAND_PIN_2]=" + pin_to_str(subband_pins[2]) + "\r\n";
  content += "[SUBBAND_PIN_3]=" + pin_to_str(subband_pins[3]) + "\r\n\r\n";
  content += "// Пины питания шины (NC если не назначены)\r\n";
  content += "[BUS_PWR_SI   ]=" + pin_to_str(pin_pwr_si) + "\r\n";
  content += "[BUS_PWR_DS   ]=" + pin_to_str(pin_pwr_ds) + "\r\n";
  content += "[BUS_PWR_BM   ]=" + pin_to_str(pin_pwr_bm) + "\r\n";
  content += "[BUS_PWR_DL   ]=" + pin_to_str(pin_pwr_dl) + "\r\n\r\n";
  content += "// Исключения из сканирования шин\r\n";
  content += "[SCAN_EXCLUDE ]=" + scan_exclude_list + "\r\n\r\n";
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

  save_ram_to_flash();
  // Сообщаем ОС, что накопитель переподключен
  usb_msc.setUnitReady(false); 
  delay(1500); 
  usb_msc.setUnitReady(true); 
}


// Проверка, входит ли конкретный пин в список исключений SCAN_EXCLUDE
// ПРАВКА: Автоматическое исключение ВСЕХ назначенных в системе пинов + ручного списка
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
  // 1. Генерирует чистую структуру FAT12 в ОЗУ и сама вызывает save_ram_to_flash()
  create_default_fat_with_info_file(); 
  
  // 2. Сразу же обновляем глобальные переменные в ОЗУ из нового дефолтного файла
  read_file_to_variable();  
  read_hardware_settings(); 
  
  // 3. Жестко уведомляем Windows, чтобы он перечитал файловую систему
  usb_msc.setUnitReady(false);
  delay(1500); 
  usb_msc.setUnitReady(true);
  
  Serial.println(F("[Система] Все файлы успешно перезаписаны на дефолтные!"));
}




// добавление произвольного блока строк в LOG.TXT  
// ОДИН вызов save_ram_to_flash() и ОДИН цикл перемонтирования USB на весь блок  
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
    log_file_clear();  
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
  
  save_ram_to_flash();              // <-- один раз на весь блок  
  Serial.print("[Журнал]  Блок добавлен, объем: ");  
  Serial.println(new_size);  
  
  usb_msc.setUnitReady(false);      // <-- одно перемонтирование  
  delay(1500);  
  usb_msc.setUnitReady(true);  
}
