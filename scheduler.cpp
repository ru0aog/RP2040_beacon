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
#include "file_manager.h" // Доступ к константам и структуре TaskItem
#include "si5351_driver.h"
#include "climate_log.h"

extern int pin_amp_act;          // Динамический пин активации УМ из file_manager.cpp
extern bool dev_TX_state;        // Состояние усилителя (true = передача, false = прием)

// Прямой проброс внешнего массива матричного расписания из ОЗУ
extern TaskItem beacon_schedule[MAX_SCHEDULE_TASKS];

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
// Выделение памяти под независимый день недели
uint8_t rtc_dotw = 1;

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
    // если часы обнаружены
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
    // часы подключены
    device_DS[0] = 1;
    activeRtc = RTC_DS3231;
/*
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
*/
    update_scheduler();
    char buf[34];
    snprintf(buf, sizeof(buf), " - дата      : %02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), " - время     : %02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
    Serial.println(buf);
  }

// Serial.println("Прямой тест памяти: H=" + String(rtc_hour) + " M=" + String(rtc_min) + " S=" + String(rtc_sec));
}


// Обновление переменных времени из регистров BCD
void update_scheduler() {
  // установить состояние пина управления УМ
  // ПРАВКА: Циклический фоновый контроль состояния УМ на динамическом пине из SET.TXT
  if (pin_amp_act != -1) {
    gpio_put(pin_amp_act, dev_TX_state);
  }

  if (device_DS[0] == 1 && (activeRtc == RTC_DS3231 || activeRtc == RTC_DS1307)) {
    // если часы подключены
    TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
    
    pWire->beginTransmission(RTC_I2C_ADDRESS);
    pWire->write(0x00); 
    pWire->endTransmission();
    
    pWire->requestFrom(RTC_I2C_ADDRESS, (uint8_t)7); 
    if (pWire->available() >= 7) {
      rtc_sec   = bcd2bin(pWire->read() & 0x7F); // 0x00: Секунды
      rtc_min   = bcd2bin(pWire->read());        // 0x01: Минуты
      rtc_hour  = bcd2bin(pWire->read() & 0x3F); // 0x02: Часы
      pWire->read(); // Пропускаем день недели (регистр 0x03)
      //rtc_dotw  = bcd2bin(pWire->read() & 0x07); // 0x03: День недели (записываем сюда вместо пропуска!)
      rtc_day   = bcd2bin(pWire->read());        // 0x04: День месяца (дата)
      rtc_month = bcd2bin(pWire->read() & 0x1F); // 0x05: Месяц
      rtc_year  = bcd2bin(pWire->read()) + 2000; // 0x06: Год
    }

  }
  else {
    if (rtc_get_datetime(&currentTime)) {
      uint8_t dotw = currentTime.dotw;
      rtc_sec   = currentTime.sec;
      rtc_min   = currentTime.min;
      rtc_hour  = currentTime.hour;
      rtc_day   = currentTime.day;
      rtc_dotw = (dotw == 0) ? 7 : dotw;
      rtc_month = currentTime.month;
      rtc_year  = currentTime.year;
    }
  }
  // обновить лог климата
  climate_log_update();
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
  char buf[16]; 
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
  return String(buf);
}

// Исправлено: разделители заменены на корректные точки '.'
String get_current_date() {
  update_scheduler();
  char buf[16]; 
  snprintf(buf, sizeof(buf), "%02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
  return String(buf);
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



// scheduler.cpp — чистый парсер одной строки  
// Формат: "МОДА | ДНИ | ВРЕМЯ"  или совместимый "ДНИ,ВРЕМЯ,ЧАСТОТА,МОДА"  
// Возвращает true, если строка корректна  
static bool parse_days(const String& s, uint8_t& mask) {  
  mask = 0;  
  if (s == "0" || s.length() == 0) return true;          // каждый день  
  for (size_t i = 0; i < s.length(); i++) {  
    char c = s[i];  
    if (c >= '1' && c <= '7') mask |= (1 << (c - '0'));  
    else if (c == '-') {                                  // диапазон a-b  
      if (i == 0 || i + 1 >= s.length()) return false;  
      int a = s[i - 1] - '0', b = s[i + 1] - '0';  
      if (a < 1 || b > 7 || a > b) return false;  
      for (int d = a; d <= b; d++) mask |= (1 << d);  
    }  
    else if (c != ' ') return false;  
  }  
  return true;  
}  
  
static bool parse_hhmm(const String& s, uint8_t& h, uint8_t& m) {  
  int colon = s.indexOf(':');  
  if (colon == -1) return false;  
  int hh = s.substring(0, colon).toInt();  
  int mm = s.substring(colon + 1).toInt();  
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59) return false;  
  h = hh; m = mm;  
  return true;  
}  
  
// ВРЕМЯ: "HH:MM" | "HH:MM-HH:MM/N" | "HH:MM/HH:MM/N"  
static bool parse_time_expr(const String& s, TaskItem& t) {  
  int sep  = s.indexOf('-');  if (sep == -1) sep = s.indexOf('/');  
  int sep2 = s.indexOf('/', sep + 1);  
  if (sep == -1) {            // разовая  
    t.interval_min = 0;  
    if (!parse_hhmm(s, t.start_hour, t.start_min)) return false;  
    t.end_hour = t.start_hour; t.end_min = t.start_min;  
    return true;  
  }  
  if (sep2 == -1) return false;  
  if (!parse_hhmm(s.substring(0, sep),  t.start_hour, t.start_min)) return false;  
  if (!parse_hhmm(s.substring(sep + 1, sep2), t.end_hour, t.end_min)) return false;  
  int iv = s.substring(sep2 + 1).toInt();  
  if (iv < 1 || iv > 1440) return false;  
  t.interval_min = iv;  
  return true;  
}



/*
// Автоматический парсер текстового расписания
// ПРАВКА: Новая высокоскоростная побитовая проверка расписания с защитой от наложений
bool is_time_to_transmit(uint8_t mode) {
  extern bool is_transmitting;         
  uint32_t cur_abs_min = rtc_hour * 60 + rtc_min;

  if (mode == 0 && cur_abs_min == last_ifkp_minute) return false;
  if (mode == 1 && cur_abs_min == last_rtty_minute) return false;
  if (mode == 2 && cur_abs_min == last_cw_minute)   return false;

  for (int i = 0; i < MAX_SCHEDULE_TASKS; i++) {
    TaskItem& task = beacon_schedule[i];
    if (!task.active || task.mode != mode) continue;

    // 1. ИСПРАВЛЕНО: Сверхбыстрая проверка дня недели через наложение битовой маски
    if (task.days > 0) {
      if ((task.days & (1 << rtc_dotw)) == 0) continue; // Если бит текущего дня не взведен — пропускаем
    }

    uint32_t start_abs = task.start_hour * 60 + task.start_min;
    uint32_t end_abs   = task.end_hour * 60 + task.end_min;

    if (task.interval_min == 0) {
      if (cur_abs_min == start_abs) {
        if (is_transmitting) {
          Serial.println(F("[Планировщик] ВНИМАНИЕ: Наложение интервалов! Задача пропущена."));
          return false;
        }
        if (mode == 0) last_ifkp_minute = cur_abs_min;
        if (mode == 1) last_rtty_minute = cur_abs_min;
        if (mode == 2) last_cw_minute   = cur_abs_min;
        return true;
      }
    } else {
      if (cur_abs_min >= start_abs && cur_abs_min <= end_abs) {
        uint32_t elapsed = cur_abs_min - start_abs;
        if (elapsed % task.interval_min == 0) {
          if (is_transmitting) {
            Serial.println(F("[Планировщик] ВНИМАНИЕ: Предыдущий сеанс занял шину! Шаг пропущен."));
            return false;
          }
          if (mode == 0) last_ifkp_minute = cur_abs_min;
          if (mode == 1) last_rtty_minute = cur_abs_min;
          if (mode == 2) last_cw_minute   = cur_abs_min;
          return true;
        }
      }
    }
  }
  return false;
}
*/


// scheduler.cpp — вывод таблицы расписания в Serial  
// scheduler.cpp — вывод таблицы расписания в Serial  
void print_schedule() {  
  static const char* mode_names[] = {"IFKP", "RTTY", "CW", "SEQ"};  // по enum BeaconMode  
  
  const int DAYS_WIDTH = 18;   // ширина колонки "Дни" в СИМВОЛАХ (под "Пн,Вт,Ср,Чт,Пт")  
  const int TIME_WIDTH = 18;   // ширина колонки "Время" в символах  
  
  Serial.println(F("=== РАСПИСАНИЕ ПЕРЕДАЧ ==="));  
  Serial.println(F("No | МОДА | Дни               | Время             |    Частота"));  
  
  uint8_t found = 0;  
  for (int i = 0; i < MAX_SCHEDULE_TASKS; i++) {  
    TaskItem &t = beacon_schedule[i];  
    if (!t.active) continue;  
    found++;  
  
    // --- Дни недели: 0 = каждый день, иначе маска -> "Пн,Вт,Ср,Чт,Пт" ---  
    String days;  
    if (t.days == 0 || (t.days & 0xFE) == 0xFE) {  // 0 = каждый день; 0xFE = биты Пн..Вс — вся неделя
      days = "ежедневно";  
    } else {  
      const char* dn[] = {"", "Пн", "Вт", "Ср", "Чт", "Пт", "Сб", "Вс"};  
      for (uint8_t d = 1; d <= 7; d++) {  
        if (t.days & (1 << d)) {  
          if (days.length()) days += ",";  
          days += dn[d];  
        }  
      }  
    }  
  
    // --- Время: одиночная "15:15" или период "17:00-22:00/5мин" ---  
    char timebuf[24];  
    if (t.interval_min == 0) {  
      snprintf(timebuf, sizeof(timebuf), "%02d:%02d", t.start_hour, t.start_min);  
    } else {  
      snprintf(timebuf, sizeof(timebuf), "%02d:%02d-%02d:%02d/%dмин",  
               t.start_hour, t.start_min, t.end_hour, t.end_min, t.interval_min);  
    }  
  
    const char* mname = (t.mode <= 3) ? mode_names[t.mode] : "?";  
  
    // --- Печать строки ---  
    // Колонка "Дни" содержит кириллицу (UTF-8: 2 байта на символ),  
    // поэтому printf с %-Ns не подходит — выравниваем вручную.  
    Serial.printf("%2d | %-4s | ", i+1, mname);  
  
    Serial.print(days);  
    // UTF-8: новый символ = байт < 0x80 (ASCII) или начальный байт >= 0xC0  
    int day_chars = 0;  
    for (size_t k = 0; k < days.length(); k++) {  
      uint8_t b = (uint8_t)days[k];  
      if (b < 0x80 || b >= 0xC0) day_chars++;  
    }  
    for (int pad = day_chars; pad < DAYS_WIDTH; pad++) Serial.print(' ');  
  
    // Колонка "Время" ASCII — но "%-18s" тоже считает байты; тут кириллицы нет,  
    // кроме слова "мин". Считаем символы и допечатываем пробелы вручную.  
    Serial.print("| ");  
    Serial.print(timebuf);  
    int time_chars = 0;  
    for (size_t k = 0; k < strlen(timebuf); k++) {  
      uint8_t b = (uint8_t)timebuf[k];  
      if (b < 0x80 || b >= 0xC0) time_chars++;  
    }  
    for (int pad = time_chars; pad < TIME_WIDTH; pad++) Serial.print(' ');  
  
    Serial.printf("| %11s Гц\n", format_freq(t.freq_hz).c_str());  
  }  
  
  if (!found) Serial.println(F("(пусто — задач нет)"));  
  Serial.printf("Всего задач: %d\n", found);  
}



// scheduler.cpp — планировщик с диагностикой сработки  
bool is_time_to_transmit(uint8_t mode) {  
  extern bool is_transmitting;  
  uint32_t cur = rtc_hour * 60UL + rtc_min;  
  static uint32_t last_min[3] = {9999, 9999, 9999};  
  
  if (mode > 2) return false;  
  if (cur == last_min[mode]) return false;               // один запуск на минуту  
  
  for (int i = 0; i < MAX_SCHEDULE_TASKS; i++) {  
    TaskItem& t = beacon_schedule[i];  
    if (!t.active || t.mode != mode) continue;  
    if (t.days && !(t.days & (1 << rtc_dotw))) continue; // день не совпал  
  
    uint32_t st = t.start_hour * 60UL + t.start_min;  
    uint32_t en = t.end_hour   * 60UL + t.end_min;  
    bool hit;  
    if (t.interval_min == 0) hit = (cur == st);  
    else {  
      hit = (en >= st) ? (cur >= st && cur <= en && (cur - st) % t.interval_min == 0)  
                       : (cur >= st || cur <= en) &&                        // окно через полночь  
                         ((cur >= st ? cur - st : cur + 1440 - st) % t.interval_min == 0);  
    }  
    if (!hit) continue;  
  
    if (is_transmitting) {  
      Serial.printf("[Планировщик] TASK_%02d: шина занята, сеанс пропущен\n", i);  
      return false;  
    }  
    Serial.printf("[Планировщик] TASK_%02d: запуск mode=%d в %02d:%02d\n",  
                  i, mode, rtc_hour, rtc_min);  
    last_min[mode] = cur;  
    return true;  
  }  
  return false;  
}