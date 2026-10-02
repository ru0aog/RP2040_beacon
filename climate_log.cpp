/**
 * ============================================================================  
 *  climate_log.cpp — Менеджер периодического логгирования погоды в LOG.TXT  
 *  Версия 2.12 от 2026-10-01, автор RU0AOG  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Обеспечивает фоновое накопление климатических данных в оперативной памяти  
 *  и периодическую запись структурированных часовых таблиц в файл журнала LOG.TXT.  
 *  
 *  МЕХАНИЗМ КОЛЬЦЕВОГО БУФЕРА И ПАКЕТНАЯ ЗАПИСЬ  
 *  --------------------------------------------  
 *  Датчик опрашивается с интервалом SAMPLE_INTERVAL_MS (10 минут в боевом режиме).  
 *  Данные складываются в статический массив структур ClimateSample. Если буфер  
 *  переполняется до момента вывода отчета, функция memmove автоматически сдвигает  
 *  массив влево, удаляя самую старую запись. По истечении REPORT_INTERVAL_MS (1 час)  
 *  формируется текстовая таблица. Она сбрасывается во Flash ОДНИМ вызовом  
 *  функции `log_file_write_block`, что экономит ресурс ячеек памяти (Wear Leveling).  
 *  
 *  ЗАЩИТА СЕССИЙ СВЯЗИ  
 *  -------------------  
 *  climate_log_update() вызывается в loop() каждую итерацию. Если глобальный  
 *  флаг `is_transmitting` равен true (маяк находится в эфире), автомат логгера  
 *  сбрасывает счетчик замеров, очищает буфер и принудительно сдвигает таймеры  
 *  вперед. Это полностью исключает внезапное I2C-обращение к BME280 посреди  
 *  передачи и предотвращает джиттер частоты радиосигнала.  
 * ============================================================================  
 */



// climate_log.cpp — периодический сбор климата и часовая таблица в LOG.TXT  
#include "bme280.h"  
#include "scheduler.h"  
#include "file_manager.h"


// ---------- Периоды ----------  
// Отладка: замер каждые 10 сек, таблица каждые 5 мин  
// Боевой режим: замер каждые 10 мин, таблица каждый час  
#define CLIMATE_DEBUG 0       // 0 — отключить быстрый тестовый опрос
#define DEBUG_CLIMATE_LOG 0   // 0 — отключить вывод в порт    
  
#if CLIMATE_DEBUG  
  #define SAMPLE_INTERVAL_MS   (10UL * 1000UL)        // 10 с  
  #define REPORT_INTERVAL_MS   (5UL * 60UL * 1000UL)  // 5 мин  
  #define MAX_SAMPLES          32   // 5 мин / 10 с = 30 замеров + запас  
#else  
  #define SAMPLE_INTERVAL_MS   (10UL * 60UL * 1000UL) // 10 мин  
  #define REPORT_INTERVAL_MS   (60UL * 60UL * 1000UL) // 1 час  
  #define MAX_SAMPLES          8    // час / 10 мин = 6 замеров + запас  
#endif  
  
struct ClimateSample {  
  char   time[9];   // "HH:MM:SS"  
  float  temp;  
  float  press;     // мм рт. ст.  
  float  humid;     // % (0 для BMP280/BMP180)  
};  
  
static ClimateSample samples[MAX_SAMPLES];  
static uint8_t  sample_count   = 0;  
static uint32_t last_sample_ms = 0;  
static uint32_t last_report_ms = 0;  
  
// Снимок датчика в кольцевой буфер  
static void climate_take_sample() {  
  // датчика нет — нечего писать  
    if (device_BM[0] != 1) {  
  #if DEBUG_CLIMATE_LOG  
      Serial.println(F("[Климат] Замер пропущен: датчик не обнаружен"));  
  #endif  
      return;  
    }
  
  get_BME_data();                // обновляет bme_temp / bme_press / bme_humid  
  
  if (sample_count >= MAX_SAMPLES) {  
    // буфер полон до вывода таблицы — сдвигаем, выбрасывая самый старый  
    memmove(&samples[0], &samples[1], sizeof(ClimateSample) * (MAX_SAMPLES - 1));  
    sample_count = MAX_SAMPLES - 1;  
  }  
  
  ClimateSample &s = samples[sample_count];  
  snprintf(s.time, sizeof(s.time), "%s", get_current_time().c_str());  
  s.temp  = BME_temp;  
  s.press = BME_press;  
  s.humid = BME_humid;  
  sample_count++;  
  
#if DEBUG_CLIMATE_LOG  
  Serial.printf("[Климат] Замер %2d: %s  T=%.1f C  P=%.1f мм  H=%.1f %%\n",  
                sample_count, s.time, s.temp, s.press, s.humid);  
#endif  
}
  
static void climate_write_report() {  
  if (sample_count == 0) {  
#if DEBUG_CLIMATE_LOG  
    Serial.println(F("[Климат] Отчёт пропущен: буфер пуст"));  
#endif  
    return;  
  }  
  
#if DEBUG_CLIMATE_LOG  
  Serial.printf("[Климат] Формирую таблицу: %d замеров -> LOG.TXT... \r\n", sample_count);  
#endif  
  
  String table;  
  table.reserve(sample_count * 48 + 96);  
  table += "=== КЛИМАТ за " + get_current_date() + " ===\r\n";  
  table += "TIME      T(C)   P(mm)  H(%)\r\n";  
  for (uint8_t i = 0; i < sample_count; i++) {  
    char line[48];  
    snprintf(line, sizeof(line), "%s  %5.1f  %6.1f  %4.1f\r\n",  
             samples[i].time, samples[i].temp, samples[i].press, samples[i].humid);  
    table += line;  
  }  
  
  log_file_write_block(table);   // одна запись во Flash на всю таблицу  
  sample_count = 0;  
  
#if DEBUG_CLIMATE_LOG  
  Serial.println(F("[Климат] готово"));  
#endif  
}


// Проверка времени
// Вызывать из loop() каждую итерацию  
void climate_log_update() {  
  uint32_t now = millis();  
  
  // Пауза на время сеанса передачи: сдвигаем оба таймера,  
  // чтобы не было мгновенного догоняющего замера после эфира  
  if (is_transmitting) {  
    last_sample_ms = now;  
    last_report_ms = now;
    sample_count = 0;
    return;  
  }  
  
  if (now - last_sample_ms >= SAMPLE_INTERVAL_MS) {  
    last_sample_ms = now;  
    climate_take_sample();  
  }  
  
  if (now - last_report_ms >= REPORT_INTERVAL_MS) {  
    last_report_ms = now;  
    climate_write_report();  
  }  
}