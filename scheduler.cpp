/**  
 * ============================================================================  
 *  scheduler.cpp — Часы реального времени (RTC) и планировщик передач маяка  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Хранит текущее время/дату и решает, когда маяку выходить в эфир. Работает  
 *  напрямую с регистрами чипов RTC по I2C (без сторонних библиотек), а при  
 *  отсутствии внешнего чипа — с внутренним календарём RP2040.  
 *  
 *  ТРИ ИСТОЧНИКА ВРЕМЕНИ (автоопределение при init_scheduler)  
 *  ---------------------------------------------------------  
 *  1. DS3231 (высокоточный, с датчиком температуры) — определяется по I2C.  
 *  2. DS1307A (стандартный) — распознаётся по тестовому биту 0x70 регистра 0x0F;  
 *     при необходимости запускается его осциллятор (сброс бита CH в 0x00).  
 *  3. Внутренний RTC RP2040 (резерв, RTC_INTERNAL) — если чип на шине не найден  
 *     (pingRTC). При этом clk_rtc тактируется от PLL_SYS, а clk_peri/АЦП  
 *     принудительно возвращаются на PLL_SYS для стабильной телеметрии.  
 *  Тип активного источника хранится в enum RtcType (activeRtc).  
 *  
 *  ФОРМАТ ДАННЫХ  
 *  -------------  
 *  Регистры внешних чипов — в BCD; конвертация bcd2bin/bin2bcd. Время/дата  
 *  публикуются в глобальные rtc_hour/min/sec/year/month/day (update_scheduler).  
 *  Шина выбирается динамически: Wire или Wire1 по device_DS[1], пины SDA/SCL —  
 *  из device_DS[2]/[3], адрес чипа — RTC_I2C_ADDRESS (0x68).  
 *  
 *  ПЛАНИРОВЩИК  
 *  -----------  
 *  is_time_to_transmit(mode) разбирает текстовое расписание вида "ЧЧ:ММ,ЧЧ:ММ,..."  
 *  из внешних строк my_ifkp_variable / my_rtty_variable / my_cw_variable  
 *  (mode 0=IFKP, 1=RTTY, 2=CW). Защита last_*_minute не даёт повторного запуска  
 *  в ту же минуту; команды time/date сбрасывают её (значение 9999).  
 *  
 *  ТЕЛЕМЕТРИЯ  
 *  ----------  
 *  get_telemetry_string() читает температуру DS3231 (рег. 0x11) и температуру  
 *  кристалла RP2040 через АЦП (канал 4). На время замера шина I2C часов  
 *  останавливается, пины 26/27 временно переводятся в HIGH для стабилизации  
 *  опорного узла АЦП, затем шина восстанавливается (I2C_DS_restart).  
 *  
 *  ПУБЛИЧНЫЙ API  
 *  -------------  
 *  init_scheduler()           — обнаружение чипа и запуск источника времени.  
 *  update_scheduler()         — обновление глобальных переменных времени.  
 *  get_current_time/date()    — строки времени/даты.  
 *  print_current_time/date()  — вывод в Serial.  
 *  get_telemetry_string()     — строка "T_DS=.. T_CPU=..".  
 *  is_time_to_transmit(mode)  — проверка расписания для режима.  
 *  handle_time_command / handle_date_command — установка времени/даты из UART.  
 * ============================================================================  
 */

#include <hardware/adc.h> // для работы с АЦП RP2040
#include <hardware/rtc.h>
#include <hardware/clocks.h>
#include <pico/util/datetime.h>
#include <Wire.h>
#include "scheduler.h"

// Перечисление для типов подключенных чипов времени
RtcType activeRtc = RTC_NONE;

datetime_t currentTime;

// Глобальные переменные для хранения текущего времени
uint8_t  rtc_hour  = 0;
uint8_t  rtc_min   = 0;
uint8_t  rtc_sec   = 0;
uint16_t rtc_year  = 0;
uint8_t  rtc_month = 0;
uint8_t  rtc_day   = 0;

// Переменные для предотвращения повторных запусков в одну и ту же минуту
static uint32_t last_ifkp_minute = 9999; 
static uint32_t last_rtty_minute = 9999;
static uint32_t last_cw_minute = 9999;

// Вспомогательные функции конвертации BCD (Binary Coded Decimal) формата чипов RTC
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }

void I2C_DS_restart() {
  // перезапуск шины Wire на линиях часов
  if (device_DS[4] == RTC_I2C_ADDRESS) {
    // если датчик обнаружен
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
    // Настройка пинов и старт шины I2C
    pWire->end();
    pWire->setSDA(device_DS[2]);
    pWire->setSCL(device_DS[3]);
    pWire->begin();
    pWire->setClock(400000);
    //Serial.print("[Система] Рестарт шины часов\n");
  }
}

// Низкоуровневая проверка: отвечает ли чип по I2C
bool pingRTC() {
  // перезапуск шины Wire на линиях часов
  if (device_DS[4] == RTC_I2C_ADDRESS) {
    TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
    pWire->beginTransmission(RTC_I2C_ADDRESS);
    //Serial.println("[Система] пинг модуля RTC");
    return (pWire->endTransmission() == 0);
  }
return 0;
}

// Аппаратная инициализация часов без библиотек
void init_scheduler() {
  I2C_DS_restart(); // Настраиваем шину пинов
  
  TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
  pWire->setTimeout(50);
  
  if (!pingRTC()) {
    Serial.println("[Система] ОШИБКА! Внешний модуль RTC не найден на шине.");
    activeRtc = RTC_INTERNAL;
    device_DS[0] = 0;
    device_DS[4] = 0;
    
    Serial.print("[Система] Запуск собственных часов чипа RP2040: ");

    #if defined(ARDUINO_ARCH_RP2040)
    uint32_t clk_sys_hz = rp2040.f_cpu(); // Добавлены скобки вызова функции ()
    
    // 1. Аппаратно переключаем источник и настраиваем безопасное тактирование модуля RTC от PLL_SYS
    clock_configure(clk_rtc,
                    0, 
                    CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // Заменено на верную константу ядра
                    clk_sys_hz, 
                    clk_sys_hz / 46875); // Корректный делитель для получения необходимой сетки частот RTC
    // 2. [КОРРЕКЦИЯ КАЛИБРОВКИ]: Принудительно восстанавливаем частоту периферии clk_peri.
    // Это намертво фиксирует частоту АЦП, убирая разбег цифр между режимами!
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    clk_sys_hz,
                    clk_sys_hz);
    #endif
    // 3. Заново инициализируем модуль АЦП на восстановленной частоте
    adc_init(); 
    delay(10);

    // Теперь вызов инициализации встроенного календаря абсолютно безопасен и не вызовет зависания
    rtc_init();
    delay(10); 
    
    datetime_t setcurrentTime = {
      .year  = 2026,
      .month = 7,
      .day   = 27,
      .dotw  = 1, 
      .hour  = 15,
      .min   = 00,
      .sec   = 45};
      
    // Безопасная аппаратная установка времени во внутренний регистр
    rtc_set_datetime(&setcurrentTime);
    delay(10); 
    
    update_scheduler();

    Serial.print(get_current_time());
    Serial.print(" ");
    Serial.println(get_current_date());
    return; // МГНОВЕННЫЙ ВЫХОД, к I2C больше не прикасаемся!
  } else {
    device_DS[0] = 1;

    // Автоопределение типа чипа часов (DS3231 vs DS1307A)
    pWire->beginTransmission(RTC_I2C_ADDRESS);
    pWire->write(0x0F);
    pWire->write(0x70); 
    
    if (pWire->endTransmission() == 0) {
      pWire->beginTransmission(RTC_I2C_ADDRESS);
      pWire->write(0x0F);
      pWire->endTransmission();
      
      uint8_t rxBytes = pWire->requestFrom(RTC_I2C_ADDRESS, (uint8_t)1);
      uint8_t testByte = 0;
      if (rxBytes > 0 && pWire->available()) {
        testByte = pWire->read();
      }

      if ((testByte & 0x70) == 0x70) {
        activeRtc = RTC_DS1307;
        Serial.println("[Система] Обнаружен стандартный чип DS1307A");
        
        pWire->beginTransmission(RTC_I2C_ADDRESS);
        pWire->write(0x0F);
        pWire->write(0x00);
        pWire->endTransmission();
      } else {
        activeRtc = RTC_DS3231;
        Serial.println("[Система] Обнаружен чип высокой точности DS3231");
      }
    } else {
      activeRtc = RTC_DS3231; 
    }

    if (activeRtc == RTC_DS1307) {
      pWire->beginTransmission(RTC_I2C_ADDRESS);
      pWire->write(0x00);
      pWire->endTransmission();
      
      if (pWire->requestFrom(RTC_I2C_ADDRESS, (uint8_t)1) > 0 && pWire->available()) {
        uint8_t secReg = pWire->read();
        if (secReg & 0x80) { 
          pWire->beginTransmission(RTC_I2C_ADDRESS);
          pWire->write(0x00);
          pWire->write(secReg & 0x7F); 
          pWire->endTransmission();
          Serial.println("[Система] Осциллятор DS1307A успешно запущен.");
        }
      }
    }

    update_scheduler();
    char buf[34];
    snprintf(buf, sizeof(buf), " - дата      : %02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), " - время     : %02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
    Serial.println(buf);
  }
}


// Обновление переменных времени из регистров BCD
void update_scheduler() {
  if (device_DS[0] == 1 && (activeRtc == RTC_DS3231 || activeRtc == RTC_DS1307)) {
    TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
    
    pWire->beginTransmission(RTC_I2C_ADDRESS);
    pWire->write(0x00); // Стартуем с регистра 0х00 (Секунды)
    pWire->endTransmission();
    
    pWire->requestFrom(RTC_I2C_ADDRESS, (uint8_t)7); // Запрашиваем 7 байт времени
    if (pWire->available() >= 7) {
      rtc_sec   = bcd2bin(pWire->read() & 0x7F);
      rtc_min   = bcd2bin(pWire->read());
      rtc_hour  = bcd2bin(pWire->read() & 0x3F); // 24-часовой формат
      pWire->read(); // Пропускаем день недели (регистр 0x03)
      rtc_day   = bcd2bin(pWire->read());
      rtc_month = bcd2bin(pWire->read() & 0x1F);
      rtc_year  = bcd2bin(pWire->read()) + 2000;
    }
  }
  else {
    // Резервный режим: Внутренний RTC чипа RP2040
    if (rtc_get_datetime(&currentTime)) {
      rtc_year  = currentTime.year;
      rtc_month = currentTime.month;
      rtc_day   = currentTime.day;
      rtc_hour  = currentTime.hour;
      rtc_min   = currentTime.min;
      rtc_sec   = currentTime.sec;
    }
  }
}

void print_current_time() {
  update_scheduler();
  char buf[34];
  snprintf(buf, sizeof(buf), "Время     : %02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
  Serial.println(buf);
}

void print_current_date() {
  update_scheduler();
  char buf[34];
  snprintf(buf, sizeof(buf), "Дата      : %02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
  Serial.println(buf);
}

String get_current_time() {
  update_scheduler();
  char buf[9]; 
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
  return String(buf);
}

// Исправлено: разделители заменены на корректные точки '.'
String get_current_date() {
  update_scheduler();
  char buf[11]; 
  snprintf(buf, sizeof(buf), "%02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
  return String(buf);
}

// Автоматический парсер текстового расписания
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
    while (*p_sched == ' ' || *p_sched == '\t') p_sched++;
    if (*p_sched == '\0') break;

    const char* token_start = p_sched;
    char* end_ptr;
    long sch_hour = strtol(p_sched, &end_ptr, 10);
    
    if (end_ptr != p_sched && *end_ptr == ':') {
      p_sched = end_ptr + 1; 
      long sch_min = strtol(p_sched, &end_ptr, 10);
      
      if (end_ptr != p_sched) {
        if (current_absolute_minutes == (uint32_t)(sch_hour * 60 + sch_min)) {
          if (mode == 0)      last_ifkp_minute = current_absolute_minutes;
          else if (mode == 1) last_rtty_minute = current_absolute_minutes;
          else if (mode == 2) last_cw_minute   = current_absolute_minutes;
          return true; 
        }
      }
      p_sched = end_ptr; 
    } else {
      p_sched = end_ptr; 
    }

    if (p_sched == token_start) {
      p_sched++;
    }
    
    while (*p_sched != '\0' && *p_sched != ',') {
      p_sched++;
    }
    
    if (*p_sched == ',') {
      p_sched++; 
    }
  }
  return false;
}

// Безопасное чтение телеметрии с проца и часов (при наличии температурного датчика)
String get_telemetry_string() {
  float rtc_temp = 0.0f;

  // Считываем температуру только если чип определен как DS3231
  if (device_DS[0] == 1 && activeRtc == RTC_DS3231) {
    TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
    pWire->beginTransmission(RTC_I2C_ADDRESS);
    pWire->write(0x11); // Регистр MSB температуры DS3231
    pWire->endTransmission();
    
    pWire->requestFrom(RTC_I2C_ADDRESS, (uint8_t)2);
    if (pWire->available() >= 2) {
      int8_t msb = pWire->read();
      uint8_t lsb = pWire->read();
      rtc_temp = msb + ((lsb >> 6) * 0.25f);
    }
  }

  // Полностью останавливаем аппаратную шину I2C часов, чтобы перехватить контроль над пинами
  TwoWire *pWireActive = (device_DS[1] == 1) ? &Wire1 : &Wire;
  pWireActive->end();

  // Переводим пины 26 и 27 в режим обычных цифровых выходов
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);

  // Жестко выставляем на них логическую единицу (+3.3 В).
  // Это принудительно запитает «висящий» опорный узел АЦП и уберет утечки!
  digitalWrite(26, HIGH);
  digitalWrite(27, HIGH);
  delayMicroseconds(50); // Даем время потенциалу стабилизироваться на отметке 3.3 В

  // Включаем и настраиваем датчик температуры процессора (Канал 4)
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4); 
  delayMicroseconds(20); 
  
  // Холостые сбросы конденсатора
  for(int h = 0; h < 10; h++) {
    ::adc_read();
    delayMicroseconds(2);
  }

  // Выполняем чистый замер температуры кристалла
  uint32_t raw_temp = 0;
  for(int i = 0; i < 20; i++) {
    raw_temp += ::adc_read(); 
    delayMicroseconds(2);
  }
  
  float voltage_temp = (raw_temp / 20.0f) * 3.3f / 4095.0f;
  float mcu_temp = 27.0f - (voltage_temp - 0.706f) / 0.001721f;

  // === ВОССТАНОВЛЕНИЕ РАБОТЫ ШИНЫ ЧАСОВ ===
  // Возвращаем пины под управление I2C-контроллера Wire1/Wire
  I2C_DS_restart();

  char tele_buf[48];
  if (device_DS[0] == 1 && activeRtc == RTC_DS3231) {
    snprintf(tele_buf, sizeof(tele_buf), "T_DS=%.1fC T_CPU=%.1fC", rtc_temp, mcu_temp);
  } else {
    snprintf(tele_buf, sizeof(tele_buf), "T_DS=N/A T_CPU=%.1fC", mcu_temp);
  }
  
  return String(tele_buf);
}

// Установка даты прямым заполнением регистров в BCD
void handle_date_command(String cmd) {
  int space_idx = cmd.indexOf(' ');
  if (space_idx == -1) return;
  
  String date_part = cmd.substring(space_idx + 1);
  date_part.trim();
  
  int first_dot = date_part.indexOf('.');
  int second_dot = date_part.indexOf('.', first_dot + 1);
  if (first_dot == -1 || second_dot == -1) {
    Serial.println("[Ошибка] Неверный формат. Используйте: date ДД.ММ.ГГГГ");
    return;
  }
  
  int d = date_part.substring(0, first_dot).toInt();
  int m = date_part.substring(first_dot + 1, second_dot).toInt();
  int y = date_part.substring(second_dot + 1).toInt();
  
  if (y >= 2000 && y < 2100 && m >= 1 && m <= 12 && d >= 1 && d <= 31) {
    if (device_DS[0] == 1 && (activeRtc == RTC_DS3231 || activeRtc == RTC_DS1307)) {
      TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
      
      // Запись новых параметров в календарную сетку чипа
      pWire->beginTransmission(RTC_I2C_ADDRESS);
      pWire->write(0x04); // Начиная с регистра 0x04 (Дата/День месяца)
      pWire->write(bin2bcd(d));
      pWire->write(bin2bcd(m));
      pWire->write(bin2bcd(y - 2000));
      pWire->endTransmission();
      Serial.println("[Система] : Календарь внешнего RTC успешно обновлен.");
    }
    else {
      // DS3231/DS1307 отсутствует — настраиваем внутренние часы RP2040
      datetime_t current_time;
      // Считываем актуальные данные из встроенного RTC
      if (rtc_get_datetime(&current_time)) {
        current_time.year  = y;
        current_time.month = m;
        current_time.day   = d;
        // Записываем обновленную структуру обратно в контроллер RTC
        rtc_set_datetime(&current_time);
        Serial.println("[Система] : Календарь встроенного RTC успешно обновлен.");
      }
    }
    // Принудительно сбрасываем защиты минут, чтобы новые параметры применились мгновенно
    last_ifkp_minute = 9999;
    last_rtty_minute = 9999;
    last_cw_minute   = 9999;
    
    print_current_date(); 
  } else {
    Serial.println("[Ошибка] Недопустимые значения дня, месяца или года.");
  }
}

// Установка времени прямым заполнением регистров в BCD
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
  } else {
    m = time_part.substring(first_colon + 1, second_colon).toInt();
    s = time_part.substring(second_colon + 1).toInt();
  }
  
  if (h >= 0 && h < 24 && m >= 0 && m < 60 && s >= 0 && s < 60) {
    if (device_DS[0] == 1 && (activeRtc == RTC_DS3231 || activeRtc == RTC_DS1307)) {
      TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
      
      // Записываем данные напрямую в регистры BCD начиная с 0x00
      pWire->beginTransmission(RTC_I2C_ADDRESS);
      pWire->write(0x00);              // Адрес первого регистра (Секунды)
      pWire->write(bin2bcd(s));        // Регистр 0x00: Секунды
      pWire->write(bin2bcd(m));        // Регистр 0x01: Минуты
      pWire->write(bin2bcd(h) & 0x3F); // Регистр 0x02: Часы (маска 0x3F гарантирует 24-часовой формат)
      pWire->endTransmission();
      
      Serial.println("[Система] : Время внешнего RTC успешно обновлено.");
    }
    else {
      // Внешний RTC отсутствует — настраиваем внутренние часы RP2040
      datetime_t current_time;
      if (rtc_get_datetime(&current_time)) {
        current_time.hour = h;
        current_time.min  = m;
        current_time.sec  = s;
        rtc_set_datetime(&current_time);
        Serial.println("[Система] : Время встроенного RTC успешно обновлено.");
      }
    }
    // Сбрасываем все три минуты блокировок для мгновенного обновления расписания
    last_ifkp_minute = 9999;
    last_rtty_minute = 9999;
    last_cw_minute   = 9999;
    
    update_scheduler();
    print_current_time(); 
  } else {
    Serial.println("[Ошибка] Недопустимые значения часов, минут или секунд.");
  }
}

