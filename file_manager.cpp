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
#include <Adafruit_TinyUSB.h>
#include <hardware/flash.h>
#include <hardware/sync.h>

// ФИЗИЧЕСКОЕ ОПРЕДЕЛЕНИЕ ОБЪЕКТОВ ДЛЯ ЛИНКОВЩИКА
Adafruit_USBD_MSC usb_msc;
String my_call_variable = "";
String my_qth_variable  = "";
String my_text_variable = "";
String my_rtty_variable = "";
String my_ifkp_variable = "";
String my_cw_variable   = "";
String my_freq_cw_var   = "";
String my_cw_wpm_var    = "";
String my_rtty_baud_var = "";
String my_rtty_mark_var = "";
String my_rtty_shift_var = ""; 
String my_rtty_invert_var = "";
String my_freq_ifkp_var  = "";
uint32_t CW_DOT_TIME_MS  = 60;            // Время точки в мс (по умолчанию ~20 WPM)
volatile uint32_t RTTY_BIT_TIME_US = 22000; // Время одного бита RTTY в мкс (по умолчанию 45.45 Бод)
String my_FAT = "";
// Физическое выделение памяти под инженерные переменные железа
int pin_freq_out = 14;  // Значения по умолчанию, если теги не найдены
int pin_amp_act  = 15;
int subband_pins[4] = {2, 3, 4, 5};
int pin_pwr_si   = -1;  // -1 означает NC (Не назначен / Not Connected)
int pin_pwr_ds   = -1;
int pin_pwr_bm   = -1;
int pin_pwr_dl   = -1;
String scan_exclude_list = "";
String scan_result_data  = "";


// Переменные трекинга текущего состояния Wear Leveling
int32_t current_active_slot = -1;
uint32_t current_max_seq = 0;

volatile bool pc_file_written = false;

// Определение параметров геометрии диска в ОЗУ
#define SECTOR_SIZE        512
#define SECTOR_COUNT       256   // 128 КБ
#define DISK_SIZE_BYTES    (SECTOR_COUNT * SECTOR_SIZE)
#define FLASH_TARGET_OFFSET (FS_START - 0x10000000)

// Выделение памяти под буфер диска в ОЗУ
alignas(4) static uint8_t ram_disk_buffer[DISK_SIZE_BYTES];

// Переменные времени для отслеживания ПК
volatile uint32_t last_msc_write_time = 0;
bool pc_activity_detected = false;

// Колбэки и посредники для TinyUSB MSC
void msc_flush_cb(void) {
  last_msc_write_time = millis();
  pc_activity_detected = true;
  pc_file_written = true; // Выставляем флаг мгновенно для экстренного останова передачи
}

int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
  if (lba >= SECTOR_COUNT) return -1;
  memcpy(buffer, &ram_disk_buffer[lba * SECTOR_SIZE], bufsize);
  return bufsize;
}

int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  if (lba >= SECTOR_COUNT) return -1;
  memcpy(&ram_disk_buffer[lba * SECTOR_SIZE], buffer, bufsize);
  return bufsize;
}

// Внутренняя функция сохранения ОЗУ во Flash с ротацией по 8 слотам
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



// Внутренняя функция генерации расширенной структуры диска FAT12 под 3 файла
static void create_default_fat_with_info_file() {
  Serial.println("[Система] Генерируем расширенный диск FAT12 (INFO, SET, LOG)...");
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
  ram_disk_buffer[17] = 48; // Увеличено количество записей в каталоге до 48 (для 3 файлов с запасом)                         
  ram_disk_buffer[18] = 0;
  ram_disk_buffer[19] = (uint8_t)(SECTOR_COUNT & 0xFF);        
  ram_disk_buffer[20] = (uint8_t)((SECTOR_COUNT >> 8) & 0xFF); 
  ram_disk_buffer[21] = 0xF8;                                  
  ram_disk_buffer[22] = 1;                                     
  ram_disk_buffer[23] = 0;
  ram_disk_buffer[24] = 0x01; ram_disk_buffer[25] = 0x00;     
  ram_disk_buffer[26] = 0x01; ram_disk_buffer[27] = 0x00;     
  ram_disk_buffer[36] = 0x80;                                  
  ram_disk_buffer[38] = 0x29;                                  
  ram_disk_buffer[39] = 0xDE; ram_disk_buffer[40] = 0xAD; ram_disk_buffer[41] = 0xBE; ram_disk_buffer[42] = 0xEF;
  memcpy(&ram_disk_buffer[43], "PICO DRIVE ", 11);               
  memcpy(&ram_disk_buffer[54], "FAT12   ", 8);                 
  ram_disk_buffer[510] = 0x55; ram_disk_buffer[511] = 0xAA;

  // СЕКТОР 1: Автоматический расчет динамической таблицы FAT12
  uint32_t fat_offset = SECTOR_SIZE * 1;
  ram_disk_buffer[fat_offset + 0] = 0xF8; 
  ram_disk_buffer[fat_offset + 1] = 0xFF; 
  ram_disk_buffer[fat_offset + 2] = 0xFF; 

  // Динамически связываем цепочку кластеров для INFO.TXT (кластеры 2..21)
  for (uint16_t i = INFO_FIRST_CLUSTER; i < (INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1); i++) {
    set_fat12_entry(fat_offset, i, i + 1); 
  }
  set_fat12_entry(fat_offset, (INFO_FIRST_CLUSTER + INFO_CLUSTERS - 1), 0xFFF); 

  // Динамически связываем цепочку кластеров для SET.TXT (кластеры 22..41)
  for (uint16_t i = SET_FIRST_CLUSTER; i < (SET_FIRST_CLUSTER + SET_CLUSTERS - 1); i++) {
    set_fat12_entry(fat_offset, i, i + 1); 
  }
  set_fat12_entry(fat_offset, (SET_FIRST_CLUSTER + SET_CLUSTERS - 1), 0xFFF); 

  // Динамически связываем цепочку кластеров для LOG.TXT (кластеры 42..241)
  for (uint16_t i = LOG_FIRST_CLUSTER; i < (LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1); i++) {
    set_fat12_entry(fat_offset, i, i + 1);
  }
  set_fat12_entry(fat_offset, (LOG_FIRST_CLUSTER + LOG_CLUSTERS - 1), 0xFFF); 

  // СЕКТОР 2: Корневой каталог (Записи по 32 байта)
  uint32_t root_offset = SECTOR_SIZE * 2;

  // 1. Запись для INFO.TXT (Смещение 0)
  memcpy(&ram_disk_buffer[root_offset + 0], "INFO    ", 8);  
  memcpy(&ram_disk_buffer[root_offset + 8], "TXT", 3);       
  ram_disk_buffer[root_offset + 26] = INFO_FIRST_CLUSTER;    

  const char* default_info_content = 
    "[CALL]=RU0AOG\r\n[QTH]=NO66FC\r\n[TEXT]=TESTING BEACON\r\n\r\n"
    "[START_CW  ]=15:15,17:15\r\n[START_RTTY]=15:18,17:18\r\n[START_IFKP]=15:20,17:20\r\n\r\n"
    "[CW_WPM    ]=20\r\n[RTTY_SPEED]=45\r\n\r\n"
    "[FREQ_CW   ]=3601500\r\n[RTTY_MARK ]=3601585\r\n[RTTY_SHIFT]=170\r\n"
    "[RTTY_INVERT]=0\r\n[FREQ_IFKP ]=3601307\r\n[EOF]";
  
  uint32_t info_len = strlen(default_info_content);
  ram_disk_buffer[root_offset + 28] = (uint8_t)(info_len & 0xFF);
  ram_disk_buffer[root_offset + 29] = (uint8_t)((info_len >> 8) & 0xFF);

  // 2. Запись для SET.TXT (Смещение 32)
  uint32_t set_entry_offset = root_offset + 32;
  memcpy(&ram_disk_buffer[set_entry_offset + 0], "SET     ", 8);  
  memcpy(&ram_disk_buffer[set_entry_offset + 8], "TXT", 3);       
  ram_disk_buffer[set_entry_offset + 26] = SET_FIRST_CLUSTER;    

  const char* default_set_content = 
    "=== ENGINEERING HARDWARE SETTINGS ===\r\n"
    "[PIN_FREQ_OUT ]=14\r\n"
    "[PIN_AMP_ACT  ]=15\r\n\r\n"
    "// Пины кода поддиапазона (4 пина)\r\n"
    "[SUBBAND_PIN_0]=2\r\n"
    "[SUBBAND_PIN_1]=3\r\n"
    "[SUBBAND_PIN_2]=4\r\n"
    "[SUBBAND_PIN_3]=5\r\n\r\n"
    "// Пины питания шины (NC если не назначены)\r\n"
    "[BUS_PWR_SI   ]=6\r\n"
    "[BUS_PWR_DS   ]=7\r\n"
    "[BUS_PWR_BM   ]=NC\r\n"
    "[BUS_PWR_DL   ]=NC\r\n\r\n"
    "// Исключения из сканирования шин\r\n"
    "[SCAN_EXCLUDE ]=14,15\r\n\r\n"
    "=== BUS SCAN DATA ===\r\n"
    "[SCAN_RESULT  ]=No scan performed yet.\r\n"
    "[LAST_SCAN_TS ]=0\r\n"
    "[EOF]";
  
  uint32_t set_len = strlen(default_set_content);
  ram_disk_buffer[set_entry_offset + 28] = (uint8_t)(set_len & 0xFF);
  ram_disk_buffer[set_entry_offset + 29] = (uint8_t)((set_len >> 8) & 0xFF);

  // 3. Запись для LOG.TXT (Смещение 64)
  uint32_t log_entry_offset = root_offset + 64;
  memcpy(&ram_disk_buffer[log_entry_offset + 0], "LOG     ", 8); 
  memcpy(&ram_disk_buffer[log_entry_offset + 8], "TXT", 3);      
  ram_disk_buffer[log_entry_offset + 26] = LOG_FIRST_CLUSTER;   

  const char* default_log_content = "=== SYSTEM LOG START ===\r\nBeacon ПО запустилось корректно.\r\n";
  uint32_t log_len = strlen(default_log_content);
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(log_len & 0xFF);
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((log_len >> 8) & 0xFF);

  // ЗАПИСЬ ДАННЫХ В СЕКТОРЫ ОБЛАСТИ ДАННЫХ RAM-ДИСКА
  // Сектор 3: Данные INFO.TXT
  uint32_t info_data_offset = SECTOR_SIZE * 3;
  memcpy(&ram_disk_buffer[info_data_offset], default_info_content, info_len);

  // Сектор 23: Данные SET.TXT (3 + INFO_CLUSTERS = 3 + 20 = 23)
  uint32_t set_data_offset = SECTOR_SIZE * (3 + INFO_CLUSTERS);
  memcpy(&ram_disk_buffer[set_data_offset], default_set_content, set_len);

  // Сектор 43: Данные LOG.TXT (3 + INFO_CLUSTERS + SET_CLUSTERS = 3 + 20 + 20 = 43)
  uint32_t log_data_offset = SECTOR_SIZE * (3 + INFO_CLUSTERS + SET_CLUSTERS); 
  memcpy(&ram_disk_buffer[log_data_offset], default_log_content, log_len);

  save_ram_to_flash();
  Serial.println("[Система] Структура диска обновлена: INFO (10Кб), SET (10Кб) и LOG (100Кб) готовы!");
}




// Внутренняя функция побайтового разбора маркеров
void read_file_to_variable() {
  flash_flush_cache();
  
  // 1. Обязательно полностью обнуляем ВСЕ строки перед чтением
  my_call_variable = "";  my_qth_variable  = "";
  my_text_variable = "";
  my_rtty_variable = "";  my_ifkp_variable = "";
  my_freq_ifkp_var = "";  my_rtty_mark_var = "";
  my_rtty_shift_var = ""; my_rtty_invert_var = ""; // Новые
  my_cw_variable = "";    my_freq_cw_var = "";
  my_cw_wpm_var    = "";
  my_rtty_baud_var = "";

  for (uint32_t i = 0; i < DISK_SIZE_BYTES - 15; i++) {
    if (ram_disk_buffer[i] == '[') {
      int32_t start_idx = -1;
      String* target_str = nullptr;

      // Точный посимвольный расчет смещений (индекс конца закрывающей скобки ']')
      // CALL
      if (ram_disk_buffer[i+1] == 'C' && ram_disk_buffer[i+2] == 'A' && ram_disk_buffer[i+3] == 'L' && ram_disk_buffer[i+4] == 'L' && ram_disk_buffer[i+5] == ']') {
        start_idx = i + 6; target_str = &my_call_variable;
      }
      // QTH
      else if (ram_disk_buffer[i+1] == 'Q' && ram_disk_buffer[i+2] == 'T' && ram_disk_buffer[i+3] == 'H' && ram_disk_buffer[i+4] == ']') {
        start_idx = i + 5; target_str = &my_qth_variable;
      }
      // TEXT
      else if (ram_disk_buffer[i+1] == 'T' && ram_disk_buffer[i+2] == 'E' && ram_disk_buffer[i+3] == 'X' && ram_disk_buffer[i+4] == 'T' && ram_disk_buffer[i+5] == ']') {
        start_idx = i + 6; target_str = &my_text_variable;
      }
      // START_CW
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "START_CW", 8) == 0) {
        for(uint32_t k = i; k < i + 15; k++) {
          if(ram_disk_buffer[k] == ']') { start_idx = k + 1; break; }
        }
        target_str = &my_cw_variable;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "START_RTTY", 10) == 0) {
        for(uint32_t k = i; k < i + 15; k++) {
          if(ram_disk_buffer[k] == ']') { start_idx = k + 1; break; }
        }
        target_str = &my_rtty_variable;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "START_IFKP", 10) == 0) {
        for(uint32_t k = i; k < i + 15; k++) {
          if(ram_disk_buffer[k] == ']') { start_idx = k + 1; break; }
        }
        target_str = &my_ifkp_variable;
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

      // 2. Если маркер найден, считываем значение строго до конца строки
      if (start_idx != -1 && target_str != nullptr) {
        target_str->reserve(32);
        
        // Если сразу после скобки идет знак '=', перешагиваем его
        if (ram_disk_buffer[start_idx] == '=') {
          start_idx++;
        }

        for (uint32_t j = start_idx; j < DISK_SIZE_BYTES; j++) {
          char c = (char)ram_disk_buffer[j];

          // ЖЕСТКИЙ ОСТАНОВ: Если дошли до конца строки или встретили начало нового тега '['
          if (c == '\n' || c == '\r' || c == '[') {
            i = j - 1; // Корректируем индекс, чтобы не пропустить следующий тег
            break;
          }

          // Фильтрация данных по типам переменных
          if (target_str == &my_freq_ifkp_var || target_str == &my_rtty_mark_var || 
          target_str == &my_rtty_shift_var || target_str == &my_rtty_invert_var || target_str == &my_freq_cw_var) {
            if (c >= '0' && c <= '9') {
              *target_str += c;
            }
          } else {
            if (c >= 32) { // Для текста и таймеров берем все печатные символы
              *target_str += c;
            }
          }
        }
        target_str->trim(); // Удаляем случайные пробелы на концах
      }
    }
  }
  
  // Пересчет WPM в миллисекунды для точки
  if (my_cw_wpm_var.length() > 0) {
    int wpm = my_cw_wpm_var.toInt();
    
    // Применяем жесткие ограничения безопасности
    if (wpm < 5)   wpm = 5;   
    if (wpm > 50) wpm = 50; 
    
    // СИНХРОНИЗАЦИЯ: Обновляем строковую переменную реальным значением
    my_cw_wpm_var = String(wpm); 
    
    // Рассчитываем длительность точки
    CW_DOT_TIME_MS = 1200 / wpm; 
  } else {
    CW_DOT_TIME_MS = 60; // Дефолт (20 WPM), если тег пустой
    my_cw_wpm_var = "20"; // Записываем дефолт и в строку тоже
  }

  // Расчет длительности бита для RTTY модема
  if (my_rtty_baud_var.length() > 0) {
    float baud = my_rtty_baud_var.toFloat(); // Радиолюбительское значение может быть 45.45
    if (baud > 0.0f) {
      // Точная формула перевода Бод в микросекунды: 1 000 000 / Скорость
      RTTY_BIT_TIME_US = (uint32_t)(1000000.0f / baud);
    }
  }
}


// Функция вывода текущих настроек
void print_current_settings() {
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
  Serial.print("Инверсия [RTTY_INVERT]: "); Serial.println(my_rtty_invert_var == "1" ? "ВКЛЮЧЕНА (Mark < Space)" : "ВЫКЛЮЧЕНА (Mark > Space)");
  Serial.print("Частота  [FREQ_IFKP ]: "); Serial.print(my_freq_ifkp_var); Serial.println(" Hz");
  Serial.print("Частота  [FREQ_IFKP ]: "); Serial.print(my_freq_ifkp_var); Serial.println(" Hz");
  // Диагностика износа ячеек и активных слотов памяти
  Serial.println("--- Статистика Wear Leveling ---");
  Serial.print("Активный слот флеши : "); Serial.println(current_active_slot != -1 ? String(current_active_slot) : "Не определен");
  Serial.print("Счетчик записей (seq): "); Serial.println(current_max_seq);
  Serial.print("Физический адрес флеш: 0x"); Serial.println(0x10000000 + FLASH_TARGET_OFFSET + (current_active_slot * SLOT_SIZE), HEX);

  Serial.println("=====================================");
}



// Глобальная инициализация файлового менеджера со сканированием износа слотов
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

    if (checked_meta.magic == SLOT_MAGIC && sig_low == 0x55 && sig_high == 0xAA) {
      // Находим слот с наибольшим номером версии (seq)
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
    my_FAT = "Файловая система флэш - корректна. Загружен слот " + String(current_active_slot) + " (seq=" + String(current_max_seq) + ")";
  } else {
    // Если ни один слот не валиден, генерируем структуру диска по умолчанию
    create_default_fat_with_info_file();
    my_FAT = "Файловая система не найдена во всех слотах. Восстановлен дефолт.";
  }

  usb_msc.setCapacity(SECTOR_COUNT, SECTOR_SIZE);
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setID("RU0AOG", "Beacon", "2.0");
  usb_msc.begin();
  usb_msc.setUnitReady(true);
  read_file_to_variable();
  read_hardware_settings(); // Инициализация пинов железа при старте
  Serial.flush();                
}



// Функция проверки изменений от ПК для loop()
void check_and_handle_pc_changes() {
  if (pc_activity_detected && (millis() - last_msc_write_time > 1500)) {
    Serial.println("\r\n> ОБНАРУЖЕНА КОРРЕКТИРОВКА INFO-ФАЙЛА");
    pc_activity_detected = false;
    
    save_ram_to_flash();
    read_file_to_variable();
    read_hardware_settings(); // Перечитываем пины, если оператор изменил SET.TXT

    usb_msc.setUnitReady(false); 
    delay(1500);                  
    usb_msc.setUnitReady(true);
    
    print_current_settings();
    pc_file_written = false; // Сбрасываем флаг только ПОСЛЕ обновления строк
  }
}


// Функция редактирования любого параметра в файле INFO.txt из консоли
// Универсальное редактирование любого параметра в файле INFO.txt из консоли
void update_info_config_from_console(String marker, String new_value) {
  uint32_t root_offset = SECTOR_SIZE * 2;
  uint32_t data_offset = SECTOR_SIZE * 3;

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
  uint32_t log_entry_offset = root_offset + 64;   // ПРАВКА: Теперь третья запись в FAT каталоге
  uint32_t log_data_offset = SECTOR_SIZE * (3 + INFO_CLUSTERS + SET_CLUSTERS); // ПРАВКА: Сектор 43

  // Стираем полностью все 100 секторов журнала в ОЗУ
  memset(&ram_disk_buffer[log_data_offset], 0, LOG_MAX_BYTES);

  const char* header = "=== ЖУРНАЛ РАБОТЫ МАЯКА ===\r\n";
  uint32_t header_len = strlen(header);
  memcpy(&ram_disk_buffer[log_data_offset], header, header_len);

  // Запись полного 4-байтного размера файла для FAT
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(header_len & 0xFF);
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((header_len >> 8) & 0xFF);
  ram_disk_buffer[log_entry_offset + 30] = 0x00;
  ram_disk_buffer[log_entry_offset + 31] = 0x00;

  save_ram_to_flash();
  Serial.println("[Журнал]  Большой файл LOG.TXT успешно очищен.");

  usb_msc.setUnitReady(false); 
  delay(1500); 
  usb_msc.setUnitReady(true);  
}

// -------------------------------------------------------------------------
// Функция дозаписи текстовой строки в конец файла LOG.TXT
// -------------------------------------------------------------------------
void log_file_write_line(String message) {
  uint32_t root_offset = SECTOR_SIZE * 2;         
  uint32_t log_entry_offset = root_offset + 64;   // ПРАВКА: Теперь третья запись в FAT каталоге
  uint32_t log_data_offset = SECTOR_SIZE * (3 + INFO_CLUSTERS + SET_CLUSTERS); // ПРАВКА: Сектор 43

  // Чтение полного 4-байтного значения размера файла из дескриптора
  uint32_t current_size = ram_disk_buffer[log_entry_offset + 28] | 
                         (ram_disk_buffer[log_entry_offset + 29] << 8) |
                         (ram_disk_buffer[log_entry_offset + 30] << 16) |
                         (ram_disk_buffer[log_entry_offset + 31] << 24);

  String formatted_msg = message + "\r\n";
  uint32_t msg_len = formatted_msg.length();

  // Сравнение с новым лимитом в 50 Килобайт
  if (current_size + msg_len >= (LOG_MAX_BYTES - 1)) {
    Serial.println("[Журнал]  Предупреждение: Лог 50 Кб заполнен! Автоматическая очистка...");
    log_file_clear(); 
    current_size = ram_disk_buffer[log_entry_offset + 28] | (ram_disk_buffer[log_entry_offset + 29] << 8);
  }

  uint8_t* write_pointer = &ram_disk_buffer[log_data_offset + current_size];
  memcpy(write_pointer, formatted_msg.c_str(), msg_len);

  uint32_t new_size = current_size + msg_len;
  // Обновление всех 4-х байт размера в оглавлении FAT
  ram_disk_buffer[log_entry_offset + 28] = (uint8_t)(new_size & 0xFF);
  ram_disk_buffer[log_entry_offset + 29] = (uint8_t)((new_size >> 8) & 0xFF);
  ram_disk_buffer[log_entry_offset + 30] = (uint8_t)((new_size >> 16) & 0xFF);
  ram_disk_buffer[log_entry_offset + 31] = (uint8_t)((new_size >> 24) & 0xFF);

  save_ram_to_flash();
  Serial.print("[Журнал]  Строка добавлена. Объем лога: "); Serial.print(new_size); Serial.println(" байт.");

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

// ПРАВКА: Функция побайтового разбора маркеров файла SET.TXT
void read_hardware_settings() {
  flash_flush_cache();
  
  // Временные строки для буферизации значений из парсера
  String s_freq_out = "", s_amp_act = "";
  String s_subband[4] = {"", "", "", ""};
  String s_pwr_si = "", s_pwr_ds = "", s_pwr_bm = "", s_pwr_dl = "";
  scan_exclude_list = "";
  scan_result_data  = "";

  for (uint32_t i = 0; i < DISK_SIZE_BYTES - 15; i++) {
    if (ram_disk_buffer[i] == '[') {
      int32_t start_idx = -1;
      String* target_str = nullptr;

      // Посимвольное сравнение ключевых маркеров файла SET.TXT
      if (strncmp((const char*)&ram_disk_buffer[i+1], "PIN_FREQ_OUT ", 12) == 0) {
        start_idx = i + 14; target_str = &s_freq_out;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "PIN_AMP_ACT  ", 12) == 0) {
        start_idx = i + 14; target_str = &s_amp_act;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SUBBAND_PIN_0", 13) == 0) {
        start_idx = i + 15; target_str = &s_subband[0];
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SUBBAND_PIN_1", 13) == 0) {
        start_idx = i + 15; target_str = &s_subband[1];
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SUBBAND_PIN_2", 13) == 0) {
        start_idx = i + 15; target_str = &s_subband[2];
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SUBBAND_PIN_3", 13) == 0) {
        start_idx = i + 15; target_str = &s_subband[3];
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "BUS_PWR_SI   ", 12) == 0) {
        start_idx = i + 14; target_str = &s_pwr_si;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "BUS_PWR_DS   ", 12) == 0) {
        start_idx = i + 14; target_str = &s_pwr_ds;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "BUS_PWR_BM   ", 12) == 0) {
        start_idx = i + 14; target_str = &s_pwr_bm;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "BUS_PWR_DL   ", 12) == 0) {
        start_idx = i + 14; target_str = &s_pwr_dl;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SCAN_EXCLUDE ", 12) == 0) {
        start_idx = i + 14; target_str = &scan_exclude_list;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "SCAN_RESULT  ", 12) == 0) {
        start_idx = i + 14; target_str = &scan_result_data;
      }

      // Если маркер обнаружен, вытаскиваем его значение до конца строки
      if (start_idx != -1 && target_str != nullptr) {
        target_str->reserve(64);
        if (ram_disk_buffer[start_idx] == '=') start_idx++;

        for (uint32_t j = start_idx; j < DISK_SIZE_BYTES; j++) {
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

  // Конвертируем считанные строки в аппаратные int-переменные
  if (s_freq_out.length() > 0) pin_freq_out = parse_pin_value(s_freq_out);
  if (s_amp_act.length()  > 0) pin_amp_act  = parse_pin_value(s_amp_act);
  
  for (int k = 0; k < 4; k++) {
    if (s_subband[k].length() > 0) subband_pins[k] = parse_pin_value(s_subband[k]);
  }
  
  if (s_pwr_si.length() > 0) pin_pwr_si = parse_pin_value(s_pwr_si);
  if (s_pwr_ds.length() > 0) pin_pwr_ds = parse_pin_value(s_pwr_ds);
  if (s_pwr_bm.length() > 0) pin_pwr_bm = parse_pin_value(s_pwr_bm);
  if (s_pwr_dl.length() > 0) pin_pwr_dl = parse_pin_value(s_pwr_dl);
}


// ПРАВКА: Автоматическая сборка структуры SET.TXT и запись её в сектор данных RAM-диска
void save_hardware_settings_to_file(String scan_results) {
  uint32_t root_offset = SECTOR_SIZE * 2;
  uint32_t set_entry_offset = root_offset + 32; // Смещение записи SET.TXT в корневом каталоге
  uint32_t set_data_offset = SECTOR_SIZE * (3 + INFO_CLUSTERS); // Начальный сектор данных файла (Сектор 23)

  // Вспомогательный лямбда-перевод числового пина в строку для конфигурации
  auto pin_to_str = [](int p) -> String {
    return (p == -1) ? "NC" : String(p);
  };

  // Динамически воссоздаем текстовое тело файла со свежими данными
  String content = "";
  content.reserve(512);
  content += "=== ENGINEERING HARDWARE SETTINGS ===\r\n";
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
  content += "=== BUS SCAN DATA ===\r\n";
  content += "[SCAN_RESULT  ]=" + scan_results + "\r\n";
  content += "[LAST_SCAN_TS ]=" + String(millis() / 1000) + "\r\n";
  content += "[EOF]";

  uint32_t total_len = content.length();
  
  // Защита от переполнения выделенного буфера в 10 КБ
  if (total_len >= SET_MAX_BYTES) {
    content = content.substring(0, SET_MAX_BYTES - 10) + "\r\n[EOF]";
    total_len = content.length();
  }

  // Очищаем старые секторы файла в ОЗУ и записываем новые данные
  memset(&ram_disk_buffer[set_data_offset], 0, SET_MAX_BYTES);
  memcpy(&ram_disk_buffer[set_data_offset], content.c_str(), total_len);

  // Пересчитываем и обновляем 4-байтный размер файла внутри оглавления FAT12
  ram_disk_buffer[set_entry_offset + 28] = (uint8_t)(total_len & 0xFF);
  ram_disk_buffer[set_entry_offset + 29] = (uint8_t)((total_len >> 8) & 0xFF);
  ram_disk_buffer[set_entry_offset + 30] = 0x00;
  ram_disk_buffer[set_entry_offset + 31] = 0x00;

  // Сбрасываем обновлённую структуру диска во флеш-память (с Wear Leveling)
  save_ram_to_flash();

  // Передёргиваем логический том TinyUSB, чтобы ПК мгновенно увидел изменения в SET.TXT
  usb_msc.setUnitReady(false);
  delay(1500);
  usb_msc.setUnitReady(true);
}

