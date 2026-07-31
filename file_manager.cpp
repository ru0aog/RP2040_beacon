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
String my_freq_ifkp_var = "";
String my_freq_cw_var   = "";
String my_rtty_space_var = "";
String my_rtty_mark_var = "";
String my_rtty_baud_var = ""; 
String my_cw_wpm_var = "";
uint32_t CW_DOT_TIME_MS = 60;            // Время точки в мс (по умолчанию ~20 WPM)
volatile uint32_t RTTY_BIT_TIME_US = 22000; // Время одного бита RTTY в мкс (по умолчанию 45.45 Бод)

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

// Внутренняя функция сохранения ОЗУ во Flash
static void save_ram_to_flash() {
  Serial.println("[Система] Сохраняем конфигурацию во Flash-память...");
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(FLASH_TARGET_OFFSET, DISK_SIZE_BYTES);
  flash_range_program(FLASH_TARGET_OFFSET, ram_disk_buffer, DISK_SIZE_BYTES);
  restore_interrupts(ints);
  flash_flush_cache();
  Serial.println("[Система] Успешно сохранено!");
}

// Внутренняя функция генерации FAT структуры по умолчанию (Совместимо с Windows 10/11)
static void create_default_fat_with_info_file() {
  Serial.println("[Система] Файлы отсутствуют. Генерируем диск и файл info.txt...");
  memset(ram_disk_buffer, 0, DISK_SIZE_BYTES);

  // -------------------------------------------------------------------------
  // СЕКТОР 0: Идеальный Boot Sector FAT12 (Совместимо с Windows 10/11)
  // -------------------------------------------------------------------------
  ram_disk_buffer[0] = 0xEB; ram_disk_buffer[1] = 0x3C; ram_disk_buffer[2] = 0x90; // Jump
  memcpy(&ram_disk_buffer[3], "MSDOS5.0", 8);                                     // OEM Name
  
  // Стандартный BIOS Parameter Block (BPB)
  ram_disk_buffer[11] = (uint8_t)(SECTOR_SIZE & 0xFF);         // Bytes per sector - Low (0x00)
  ram_disk_buffer[12] = (uint8_t)((SECTOR_SIZE >> 8) & 0xFF);  // Bytes per sector - High (0x02)
  ram_disk_buffer[13] = 1;                                     // Sectors per cluster (1)
  ram_disk_buffer[14] = 1;                                     // Reserved sectors (1)
  ram_disk_buffer[15] = 0;
  ram_disk_buffer[16] = 1;                                     // Number of FATs (1)
  ram_disk_buffer[17] = 16;                                    // Max root directory entries (16)
  ram_disk_buffer[18] = 0;
  ram_disk_buffer[19] = (uint8_t)(SECTOR_COUNT & 0xFF);        // Total sectors - Low (0x00)
  ram_disk_buffer[20] = (uint8_t)((SECTOR_COUNT >> 8) & 0xFF); // Total sectors - High (0x01)
  ram_disk_buffer[21] = 0xF8;                                  // Media descriptor (Fixed Disk)
  ram_disk_buffer[22] = 1;                                     // Sectors per FAT (1)
  ram_disk_buffer[23] = 0;
  
  // СТРОГО ПО КАРТЕ МЕСТА: Геометрия диска для Windows (Байты 24 - 35)
  ram_disk_buffer[24] = 0x01; ram_disk_buffer[25] = 0x00;     // Sectors per track (1)
  ram_disk_buffer[26] = 0x01; ram_disk_buffer[27] = 0x00;     // Number of heads (1)
  ram_disk_buffer[28] = 0x00; ram_disk_buffer[29] = 0x00;     // Hidden sectors (0)
  ram_disk_buffer[30] = 0x00; ram_disk_buffer[31] = 0x00;     
  ram_disk_buffer[32] = 0x00; ram_disk_buffer[33] = 0x00;     // Large total sectors (0)
  ram_disk_buffer[34] = 0x00; ram_disk_buffer[35] = 0x00;

  // Тот самый Extended BPB (Байты 36 - 61)
  ram_disk_buffer[36] = 0x80;                                  // Physical drive number
  ram_disk_buffer[37] = 0x00;                                  // Reserved
  ram_disk_buffer[38] = 0x29;                                  // Extended boot signature
  ram_disk_buffer[39] = 0xDE; ram_disk_buffer[40] = 0xAD;      // Volume Serial Number
  ram_disk_buffer[41] = 0xBE; ram_disk_buffer[42] = 0xEF;
  memcpy(&ram_disk_buffer[43], "PICO DRIVE ", 11);               // Volume Label (11 байт)
  memcpy(&ram_disk_buffer[54], "FAT12   ", 8);                 // File System Type (8 байт)
  
  // Сигнатура исправного загрузочного сектора на самом конце (Байты 510 и 511)
  ram_disk_buffer[510] = 0x55; 
  ram_disk_buffer[511] = 0xAA;


  // -------------------------------------------------------------------------
  // СЕКТОР 1: Таблица FAT12 (Размер: 1 сектор)
  // -------------------------------------------------------------------------
  uint32_t fat_offset = SECTOR_SIZE * 1;
  ram_disk_buffer[fat_offset + 0] = 0xF8; // Media descriptor
  ram_disk_buffer[fat_offset + 1] = 0xFF; // Клаузура заполнения FAT
  ram_disk_buffer[fat_offset + 2] = 0xFF; // Кластеры 0 и 1 зарезервированы
  
  // Данные нашего файла INFO.TXT займут Кластер 2 (Сектор 3)
  ram_disk_buffer[fat_offset + 3] = 0xFF; 
  ram_disk_buffer[fat_offset + 4] = 0x0F; 

  // -------------------------------------------------------------------------
  // СЕКТОР 2: Корневой каталог (Размер: 1 сектор)
  // -------------------------------------------------------------------------
  uint32_t root_offset = SECTOR_SIZE * 2;
  memcpy(&ram_disk_buffer[root_offset + 0], "INFO    ", 8);  // Имя файла
  memcpy(&ram_disk_buffer[root_offset + 8], "TXT", 3);       // Расширение
  ram_disk_buffer[root_offset + 11] = 0x00;                 // Атрибуты (Обычный файл)
  ram_disk_buffer[root_offset + 26] = 0x02;                 // Стартовый кластер файла = 2 (Low)
  ram_disk_buffer[root_offset + 27] = 0x00;                 // Стартовый кластер файла (High)

  const char* default_content = 
    "[CALL]=RU0AOG\r\n"
    "[QTH]=NO66FC\r\n"
    "[TEXT]=TESTING BEACON\r\n"
    "\r\n"
    "[START_CW  ]=15:15,17:15,19:15\r\n"
    "[START_RTTY]=15:18,17:18,19:18\r\n"
    "[START_IFKP]=15:20,17:20,19:20\r\n"
    "\r\n"
    "[CW_WPM    ]=20\r\n"
    "[RTTY_SPEED]=45\r\n"
    "\r\n"
    "[FREQ_CW   ]=3601500\r\n"
    "[RTTY_SPACE]=3601415\r\n"
    "[RTTY_MARK ]=3601585\r\n"
    "[FREQ_IFKP ]=3601307\r\n"
    "[EOF]";
  
  uint32_t text_len = strlen(default_content);
  ram_disk_buffer[root_offset + 28] = (uint8_t)(text_len & 0xFF);
  ram_disk_buffer[root_offset + 29] = (uint8_t)((text_len >> 8) & 0xFF);

  // -------------------------------------------------------------------------
  // СЕКТОР 3: Область данных (Кластер 2)
  // -------------------------------------------------------------------------
  uint32_t data_offset = SECTOR_SIZE * 3;
  memcpy(&ram_disk_buffer[data_offset], default_content, text_len);

  // Сохраняем свежесгенерированную структуру во Flash-память RP2040
  save_ram_to_flash();
  Serial.println("[Система] Новый шаблон info.txt успешно создан!");
}


// Внутренняя функция побайтового разбора маркеров
void read_file_to_variable() {
  flash_flush_cache();
  
  // 1. Обязательно полностью обнуляем ВСЕ строки перед чтением
  my_call_variable = "";  my_qth_variable  = "";
  my_text_variable = "";
  my_rtty_variable = "";  my_ifkp_variable = "";
  my_freq_ifkp_var = "";  my_rtty_space_var = ""; my_rtty_mark_var = "";
  my_cw_variable = "";    my_freq_cw_var = ""; my_cw_wpm_var    = "";
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
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "RTTY_SPACE]", 11) == 0) {
        start_idx = i + 12; target_str = &my_rtty_space_var;
      }
      else if (strncmp((const char*)&ram_disk_buffer[i+1], "RTTY_MARK ]", 11) == 0) {
        start_idx = i + 12; target_str = &my_rtty_mark_var;
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
          if (target_str == &my_freq_ifkp_var || target_str == &my_rtty_space_var || target_str == &my_rtty_mark_var || target_str == &my_freq_cw_var) {
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
  Serial.print("Частота  [RTTY_SPACE]: "); Serial.print(my_rtty_space_var); Serial.println(" Hz");
  Serial.print("Частота  [RTTY_MARK ]: "); Serial.print(my_rtty_mark_var); Serial.println(" Hz");
  Serial.print("Частота  [FREQ_IFKP ]: "); Serial.print(my_freq_ifkp_var); Serial.println(" Hz");
  Serial.println("=====================================");
}

// Глобальная инициализация файлового менеджера
void init_file_manager() {
  flash_flush_cache();
  memcpy(ram_disk_buffer, (const void*)(0x10000000 + FLASH_TARGET_OFFSET), DISK_SIZE_BYTES);

  if (ram_disk_buffer[510] != 0x55 || ram_disk_buffer[511] != 0xAA) {
    create_default_fat_with_info_file();
  } else {
    Serial.println("[Система] Корректный диск обнаружен во Flash. Настройки восстановлены.");
  }

  usb_msc.setCapacity(SECTOR_COUNT, SECTOR_SIZE);
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setID("RU0AOG", "Beacon", "2.0");
  usb_msc.begin();
  usb_msc.setUnitReady(true);
  read_file_to_variable();
}

// Функция проверки изменений от ПК для loop()
void check_and_handle_pc_changes() {
  if (pc_activity_detected && (millis() - last_msc_write_time > 1500)) {
    Serial.println("\r\n> ОБНАРУЖЕНА КОРРЕКТИРОВКА INFO-ФАЙЛА");
    pc_activity_detected = false;
    
    save_ram_to_flash();
    read_file_to_variable();

    usb_msc.setUnitReady(false); 
    delay(200);                  
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
