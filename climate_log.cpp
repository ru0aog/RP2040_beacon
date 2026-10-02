/**
 * ============================================================================  
 *  climate_log.cpp — Менеджер периодического логгирования погоды в LOG.TXT  
 *  Версия 2.31 (Синхронизация массивов устройств Bosch + AHT20) от 2026-10-02  
 * ============================================================================  
 */

#include "bme280.h"  
#include "scheduler.h"  
#include "file_manager.h"

// ВНЕШНИЕ ССЫЛКИ: Приводим типы к массивам uint8_t[5] строго как в bme280.cpp
extern uint8_t device_BM[5];
extern uint8_t device_AH[5];

#define MAX_SAMPLES       35

struct ClimateSample {  
  char   time[9];   // "HH:MM:SS" 
  float  temp;  
  float  press;     // мм рт. ст. 
  float  humid;     // %
};  
  
static ClimateSample samples[MAX_SAMPLES]; 
static uint8_t  sample_count   = 0; 
static uint32_t last_sample_ms = 0; 
static uint32_t last_report_ms = 0;

// --- НОВОЕ: аккумуляторы для усреднения 10 подзамеров в один замер таблицы ---  
#define SUB_SAMPLES_COUNT 10   // количество подзамеров на один замер таблицы  
  
static float   acc_temp    = 0.0f;  
static float   acc_press   = 0.0f;  
static float   acc_humid   = 0.0f;  
static uint8_t sub_count   = 0;        // сколько подзамеров уже накоплено (0..10)  
static uint32_t last_sub_sample_ms = 0; // время последнего подзамера

// Одиночный опрос датчиков — без записи в таблицу (из тела старого climate_take_sample)  
static bool climate_poll_sensors() {  
  bool has_BM = (device_BM[0] == 1);  
  bool has_AH = (device_AH[0] == 1);  
  
  if (!has_BM && !has_AH) return false;   // логировать нечего  
  
  if (has_BM) get_BME_data();  
  if (has_AH) get_AHT_data();  
  
  // Накапливаем суммы по той же логике приоритетов, что и раньше:  
  acc_press += has_BM ? BME_press : 0.0f;  
  acc_humid += has_AH ? AHT_humid : BME_humid; // влажность: приоритет AHT20  
  acc_temp  += has_BM ? BME_temp  : AHT_temp;  // температура: приоритет Bosch  
  
  sub_count++;  
  return true;  
}  
  
// Снимок в кольцевой буфер — теперь пишет ОСРЕДНЁННОЕ значение  
static void climate_take_sample() {  
  if (sub_count == 0) {              // не было ни одного удачного подзамера  
    if (debug_flag) {  
      Serial.println(F("[Климат] Замер пропущен: ни один климатический датчик не обнаружен"));  
    }  
    return;  
  }  
  
  if (sample_count >= MAX_SAMPLES) {  
    // Буфер полон до вывода таблицы — сдвигаем, выбрасывая самый старый  
    memmove(&samples[0], &samples[1], sizeof(ClimateSample) * (MAX_SAMPLES - 1));  
    sample_count = MAX_SAMPLES - 1;  
  }  
  
  ClimateSample &s = samples[sample_count];  
  snprintf(s.time, sizeof(s.time), "%s", get_current_time().c_str());  
  
  float n = (float)sub_count;  
  s.press = acc_press / n;  
  s.humid = acc_humid / n;  
  s.temp  = acc_temp  / n;  
  
  // Сброс аккумулятора для следующего цикла усреднения  
  acc_temp = acc_press = acc_humid = 0.0f;  
  sub_count = 0;  
  
  sample_count++;  
  
  if (debug_flag) {  
    Serial.printf("[Климат] Замер %2d: %s  T=%.1f C  P=%.1f мм  H=%.1f %%\n",  
                  sample_count, s.time, s.temp, s.press, s.humid);  
  }  
}


static void climate_write_report() {  
  if (sample_count == 0) { 
    if (debug_flag) {
      // вывод сообщения
      Serial.println(F("[Климат] Отчёт пропущен: буфер пуст"));
    }
    return;  
  }  
  
  if (debug_flag) {
    // вывод сообщения 
    Serial.printf("[Климат] Формирую таблицу: %d замеров -> LOG.TXT... \r\n", sample_count);
  }
  
  String table; 
  table.reserve(sample_count * 48 + 96);  
  table += "=== КЛИМАТ за " + get_current_date() + " ===\r\n"; 
  table += "TIME       T(C)   P(mm)  H(%)\r\n"; 
  for (uint8_t i = 0; i < sample_count; i++) {  
    char line[48]; 
    snprintf(line, sizeof(line), "%s  %5.1f  %6.1f  %4.1f\r\n", 
             samples[i].time, samples[i].temp, samples[i].press, samples[i].humid); 
    table += line;  
  }  
  
  log_file_write_block(table);   // Одна запись во Flash на всю таблицу 
  sample_count = 0; 
  
  if (debug_flag) {
    // вывод сообщения 
    Serial.println(F("[Климат] готово"));
  }
}



// Проверка времени (вызывается из loop() каждую итерацию)  
void climate_log_update() {    
  uint32_t now = millis();    
    
  if (is_transmitting) {    
    last_sample_ms      = now;    
    last_sub_sample_ms  = now;  
    sample_count = 0;    
    sub_count = 0;  
    acc_temp = acc_press = acc_humid = 0.0f;    
    return;    
  }    
    
  uint32_t sample_interval_ms = climate_sample_interval_min * 60UL * 1000UL;    
    
  // Подинтервал опроса датчиков — в 10 раз чаще записи в таблицу    
  uint32_t sub_interval_ms = sample_interval_ms / SUB_SAMPLES_COUNT;    
  if (sub_interval_ms == 0) sub_interval_ms = 1;  
    
  // Подзамер: опрос датчиков и накопление в аккумулятор — НЕ ТРОГАЕМ  
  if (now - last_sub_sample_ms >= sub_interval_ms) {    
    last_sub_sample_ms = now;    
    climate_poll_sensors();    
  }    
    
  // Полный замер: среднее из подзамеров -> строка таблицы  
  if (now - last_sample_ms >= sample_interval_ms) {    
    last_sample_ms = now;    
    climate_take_sample();    
  
    // ОТЧЁТ ПО ЗАПОЛНЕНИЮ: таблица уходит в LOG.TXT, когда собрано  
    // ровно CLIM_REPORT_MIN / CLIM_SAMPLE_MIN строк  
    uint8_t rows_per_report = 1;  
    if (climate_sample_interval_min > 0) {  
      uint32_t n = climate_report_interval_min / climate_sample_interval_min;  
      if (n < 1) n = 1;  
      if (n > MAX_SAMPLES) n = MAX_SAMPLES;  
      rows_per_report = (uint8_t)n;  
    }  
    if (sample_count >= rows_per_report) {  
      climate_write_report();   // собирает таблицу -> log_file_write_block() -> sample_count = 0  
    }  
  }    
}


// Сборка текущей таблицы в String (универсальный буфер)
String climate_build_current_table() {
  if (sample_count == 0) {
    return "=== КЛИМАТИЧЕСКИЙ БУФЕР ПУСТ ===";
  }

  String table;
  table.reserve(sample_count * 48 + 64);
  table += "=== КЛИМАТ ЗА " + get_current_date() + " ===\r\n";
  table += "TIME       T(C)   P(mm)  H(%)\r\n";
  for (uint8_t i = 0; i < sample_count; i++) {
    char line[48];
    snprintf(line, sizeof(line), "%s  %5.1f  %6.1f  %4.1f\r\n",
             samples[i].time, samples[i].temp, samples[i].press, samples[i].humid);
    table += line;
  }
  return table;
}

// Вывод таблицы в консоль UART
void climate_print_table_to_console() {
  Serial.println(climate_build_current_table());
}

// Принудительная отправка сформированной таблицы в эфир через планировщик
void climate_send_table_to_air() {
  if (sample_count == 0) {
    if (debug_flag) Serial.println(F("[Климат] Отмена TX: нет данных в буфере"));
    return;
  }

  // Строим таблицу (убираем спецсимволы \r, оставляя только \n для экономии времени передачи)
  String tx_table = climate_build_current_table();
  tx_table.replace("\r", ""); 

  if (debug_flag) {
    Serial.println(F("[Система] Передаю климатическую таблицу в эфир..."));
  }

  // Передаем сформированный блок текста в текущую активную моду (например, CW или RTTY)
  // Метод интеграции зависит от структуры вашего модулятора, обычно вызывается:
  // queue_text_for_transmission(tx_table);
}

