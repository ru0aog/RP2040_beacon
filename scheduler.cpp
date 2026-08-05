#include "scheduler.h"
#include <hardware/adc.h> // для работы с АЦП RP2040
#include <hardware/rtc.h>
#include <pico/util/datetime.h>

// Физическое определение объекта часов для линковщика
RTC_DS3231 rtc;
datetime_t currentTime;

bool DS_FAIL = true;
// Глобальные переменные для хранения текущего времени
uint8_t  rtc_hour  = 0;
uint8_t  rtc_min   = 0;
uint8_t  rtc_sec   = 0;
uint16_t rtc_year  = 0;
uint8_t  rtc_month = 0;
uint8_t  rtc_day   = 0;

// Переменные для предотвращения повторных запусков в одну и ту же минуту
// 255 — стартовое значение, не совпадающее ни с одной минутой суток
static uint32_t last_ifkp_minute = 9999; 
static uint32_t last_rtty_minute = 9999;
static uint32_t last_cw_minute = 9999;

void I2C_DS_restart() {
  // перезапуск шины Wire1 на линиях часов
  Wire1.end();
  Wire1.setSDA(DS_PIN_SDA);
  Wire1.setSCL(DS_PIN_SCL);
  Wire1.setClock(400000);
  Wire1.begin();
}

// Аппаратная инициализация часов
void init_scheduler() {
  pinMode(DS_POWER_PIN, OUTPUT);
  digitalWrite(DS_POWER_PIN, HIGH);
  delay(100); // Даем чипу DS3231 время на аппаратный старт
  I2C_DS_restart();
  if (!rtc.begin(&Wire1)) {
    Serial.println("[Система] КРИТИЧЕСКАЯ ОШИБКА! RTC DS-3231 не найден на шине Wire.");
    DS_FAIL = true;
    Serial.println("[Система] Запуск собственных часов чипа RP2040:");
    rtc_init();
    // 2. Устанавливаем начальное время (например: 27 июля 2026 года, 15:30:00)
    // Формат: Год, Месяц, День, Часы, Минуты, Секунды
    datetime_t setcurrentTime = {
      .year  = 2026,
      .month = 7,
      .day   = 27,
      .dotw  = 1, // День недели: 0 - Воскресенье, 1 - Понедельник и т.д.
      .hour  = 15,
      .min   = 00,
      .sec   = 45};
    rtc_set_datetime(&setcurrentTime);
    update_scheduler();
    char buf[34];
    snprintf(buf, sizeof(buf), " - дата      : %02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), " - время     : %02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
    Serial.println(buf);
    Serial.println("[Система] Встроенный RTC: ОК. RTC успешно запущен и настроен.");
  } else {
    DS_FAIL = false;
    update_scheduler();
    char buf[34];
    snprintf(buf, sizeof(buf), " - дата      : %02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), " - время     : %02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
    Serial.println(buf);
    Serial.print(F("[Система] RTC ")); 
    Serial.print(rtc_chip_name); 
    Serial.println(F(" : OK. Внешний модуль времени успешно запущен."));
  }
}

// Обновление переменных времени 
void update_scheduler() {
  if (DS_FAIL == false) {
    // из чипа DS3231
    I2C_DS_restart();
    DateTime now = rtc.now();
    rtc_year  = now.year();
    rtc_month = now.month();
    rtc_day   = now.day();
    rtc_hour  = now.hour();
    rtc_min   = now.minute();
    rtc_sec   = now.second();
  }
  else {
    // DS3231 отсутствует
    rtc_get_datetime(&currentTime);
    rtc_year  = currentTime.year;
    rtc_month = currentTime.month;
    rtc_day   = currentTime.day;
    rtc_hour  = currentTime.hour;
    rtc_min   = currentTime.min;
    rtc_sec   = currentTime.sec;
  }
}

// Вывод времени в консоль
void print_current_time() {
  update_scheduler();
  char buf[34];
  snprintf(buf, sizeof(buf), "Время     : %02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
  Serial.println(buf);
}

// Вывод только календарной даты в консоль
void print_current_date() {
  update_scheduler();
  char buf[34];
  snprintf(buf, sizeof(buf), "Дата      : %02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
  Serial.println(buf);
}

// автоматический парсер текстового расписания
bool is_time_to_transmit(uint8_t mode) {
  extern String my_ifkp_variable;
  extern String my_rtty_variable;
  extern String my_cw_variable;

  const char* p_sched = nullptr;
  if (mode == 0)      p_sched = my_ifkp_variable.c_str();
  else if (mode == 1) p_sched = my_rtty_variable.c_str();
  else if (mode == 2) p_sched = my_cw_variable.c_str();

  if (!p_sched || p_sched[0] == '\0') return false;
  
  uint32_t current_absolute_minutes = rtc_hour * 60 + rtc_min;
  
  if (mode == 0 && current_absolute_minutes == last_ifkp_minute) return false;
  if (mode == 1 && current_absolute_minutes == last_rtty_minute) return false;
  if (mode == 2 && current_absolute_minutes == last_cw_minute)   return false;

  while (*p_sched != '\0') {
    // 1. Пропускаем пробелы перед токеном времени
    while (*p_sched == ' ' || *p_sched == '\t') p_sched++;
    if (*p_sched == '\0') break;

    // Запоминаем стартовую позицию текущего токена для защиты от зависания
    const char* token_start = p_sched;

    // 2. Читаем часы
    char* end_ptr;
    long sch_hour = strtol(p_sched, &end_ptr, 10);
    
    // 3. Если встретили двоеточие — читаем минуты
    if (end_ptr != p_sched && *end_ptr == ':') {
      p_sched = end_ptr + 1; // Встаем сразу за двоеточие
      long sch_min = strtol(p_sched, &end_ptr, 10);
      
      // Если и минуты успешно прочитались, проверяем совпадение
      if (end_ptr != p_sched) {
        if (current_absolute_minutes == (uint32_t)(sch_hour * 60 + sch_min)) {
          if (mode == 0)      last_ifkp_minute = current_absolute_minutes;
          else if (mode == 1) last_rtty_minute = current_absolute_minutes;
          else if (mode == 2) last_cw_minute   = current_absolute_minutes;
          return true; 
        }
      }
      p_sched = end_ptr; // Смещаем указатель на конец успешно распарсенных минут
    } else {
      p_sched = end_ptr; // Смещаем указатель на место, где споткнулся первый strtol
    }

    // 4. ГАРАНТИРОВАННАЯ ЗАЩИТА: Если ни один strtol не смог продвинуться вперед 
    // (например, стоим на запятой, тексте или спецсимволе), принудительно делаем шаг.
    if (p_sched == token_start) {
      p_sched++;
    }
    
    // 5. Перематываем указатель строго до следующей запятой (или конца строки)
    while (*p_sched != '\0' && *p_sched != ',') {
      p_sched++;
    }
    
    // 6. Если нашли запятую — перешагиваем её для следующей итерации
    if (*p_sched == ',') {
      p_sched++; 
    }
  }

  return false;
}


// Функция для чтения телеметрии (АЦП RP2040 и температура DS3231)
String get_telemetry_string() {
  I2C_DS_restart();
  // 1. Считываем температуру с чипа DS3231
  float rtc_temp = rtc.getTemperature();

  // 2. Считываем датчик температуры самого процессора RP2040
  adc_set_temp_sensor_enabled(true); // Включаем внутренний термодатчик
  
  adc_select_input(4); // ADC4 - встроенный термодатчик
  delayMicroseconds(10); // Пауза на стабилизацию мультиплексора
  adc_read();            // ХОЛОСТОЙ ХОД: Сбрасываем остаточный заряд конденсатора АЦП

  uint32_t raw_temp = 0;
  for(int i=0; i<20; i++) {
    raw_temp += adc_read();
    delayMicroseconds(2); // Небольшой интервал между выборками
  }
  float voltage_temp = (raw_temp / 20.0f) * 3.3f / 4095.0f;
  float mcu_temp = 27.0f - (voltage_temp - 0.706f) / 0.001721f; // Формула из даташита RP2040

/*
  // 3. Считываем напряжение VSYS
  // Переключаемся на встроенный делитель VSYS
  // ВНИМАНИЕ! делитель не распаян, пока не выводим эти данные в строку
  adc_select_input(3); // ADC3 - это пин GPIO 29, измеряющий VSYS на Pico
  delayMicroseconds(10);
  adc_read(); // Холостой ход

  uint32_t raw_vbat = 0;
  for(int i=0; i<20; i++) raw_vbat += adc_read();
  float voltage_pin = (raw_vbat / 20.0f) * 3.3f / 4095.0f;

  // На плате Pico встроенный делитель делит напряжение на 3
  float v_bat = voltage_pin * 3.0f; 
*/

  // 4. Формируем компактную строчку телеметрии
  char tele_buf[48];
  snprintf(tele_buf, sizeof(tele_buf), "T_DS=%.1fC T_CPU=%.1fC", rtc_temp, mcu_temp);
  
  return String(tele_buf);
}


// Установка даты
void handle_date_command(String cmd) {
  int space_idx = cmd.indexOf(' ');
  if (space_idx == -1) return;
  
  String date_part = cmd.substring(space_idx + 1);
  date_part.trim();
  
  // Ищем первую точку между днем и месяцем
  int first_dot = date_part.indexOf('.');
  if (first_dot == -1) {
    Serial.println("[Ошибка] Неверный формат. Используйте: date ДД.ММ.ГГГГ");
    return;
  }
  
  // Ищем вторую точку между месяцем и годом
  int second_dot = date_part.indexOf('.', first_dot + 1);
  if (second_dot == -1) {
    Serial.println("[Ошибка] Неверный формат. Используйте: date ДД.ММ.ГГГГ");
    return;
  }
  
  int d = date_part.substring(0, first_dot).toInt();
  int m = date_part.substring(first_dot + 1, second_dot).toInt();
  int y = date_part.substring(second_dot + 1).toInt();
  
  // Проверяем валидность введенного календаря
  if (y >= 2000 && y < 2100 && m >= 1 && m <= 12 && d >= 1 && d <= 31) {
    if (DS_FAIL == false) {
      // из чипа DS3231
      I2C_DS_restart();
      DateTime now = rtc.now();
      rtc.adjust(DateTime(y, m, d, now.hour(), now.minute(), now.second()));
      Serial.println("[Система] : Календарная дата в чипе DS3231 успешно обновлена.");
    }
    else {
      // DS3231 отсутствует
      // Создаем пустую структуру для чтения текущего времени
      datetime_t current_time;
      // Считываем актуальные данные из встроенного RTC
      if (rtc_get_datetime(&current_time)) {
        current_time.year  = y;
        current_time.month = m;
        current_time.day   = d;
        // Записываем обновленную структуру обратно в контроллер RTC
        rtc_set_datetime(&current_time);
          Serial.println("[Система] : Календарная дата встроенного RTC успешно обновлена.");
      }
    }
    // Сбрасываем защиты
    last_ifkp_minute = 9999;
    last_rtty_minute = 9999;
    last_cw_minute   = 9999;

    print_current_date(); 
  } else {
    Serial.println("[Ошибка] Недопустимые значения дня, месяца или года.");
  }
}


// Установка времени
void handle_time_command(String cmd) {
  int space_idx = cmd.indexOf(' ');
  if (space_idx == -1) return;
  
  String time_part = cmd.substring(space_idx + 1);
  time_part.trim();
  
  int first_colon = time_part.indexOf(':');
  if (first_colon == -1) {
    Serial.println("[Ошибка] Неверный формат. Используйте: time ЧЧ:ММ или time ЧЧ:ММ:СС");
    return;
  }
  
  int h = time_part.substring(0, first_colon).toInt();
  int m = 0;
  int s = 0;
  
  int second_colon = time_part.indexOf(':', first_colon + 1);
  
  if (second_colon == -1) {
    m = time_part.substring(first_colon + 1).toInt();
    s = 0; 
  } else {
    m = time_part.substring(first_colon + 1, second_colon).toInt();
    s = time_part.substring(second_colon + 1).toInt();
  }
  
  if (h >= 0 && h < 24 && m >= 0 && m < 60 && s >= 0 && s < 60) {
    if (DS_FAIL == false) {
      // из чипа DS3231
      I2C_DS_restart();
      DateTime now = rtc.now();
      rtc.adjust(DateTime(now.year(), now.month(), now.day(), h, m, s));
      Serial.println("[Система] : Время и секунды в чипе DS3231 успешно обновлены.");
    }
    else {
      // DS3231 отсутствует
      // Создаем пустую структуру для чтения текущего времени
      datetime_t current_time;
      // Считываем актуальные данные из встроенного RTC
      if (rtc_get_datetime(&current_time)) {
        current_time.hour = h;
        current_time.min  = m;
        current_time.sec  = s;
        // Записываем обновленную структуру обратно в контроллер RTC
        rtc_set_datetime(&current_time);
          Serial.println("[Система] : Время и секунды встроенного RTC успешно обновлены.");
      }
    }
    // Принудительно сбрасываем все три защиты минут, чтобы новое время применилось мгновенно
    last_ifkp_minute = 9999;
    last_rtty_minute = 9999;
    last_cw_minute   = 9999;

    update_scheduler();
    print_current_time(); 
  } else {
    Serial.println("[Ошибка] Недопустимые значения часов, минут или секунд.");
  }
}

