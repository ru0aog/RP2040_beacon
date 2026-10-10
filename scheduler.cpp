/**  
 * ============================================================================  
 *  scheduler.cpp — Низкоуровневый драйвер RTC шины I2C и планировщик задач  
 *  Версия 2.12 от 2026-10-01, автор RU0AOG  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Обеспечивает прецизионный учет системного времени/даты и управляет запуском  
 *  сеансов связи радиомаяка по усложненным периодическим и одиночным сценариям.  
 *  
 *  ТРЕХКОНТУРНОЕ АВТООПРЕДЕЛЕНИЕ RTC И ФИКСАЦИЯ ЧАСТОТЫ АЦП  
 *  -------------------------------------------------------  
 *  Драйвер работает напрямую с регистрами I2C (адрес 0x68) без внешних библиотек:  
 *  1. Высокоточный DS3231: Выбирается базовым при успешном пинге шины.  
 *  2. Стандартный DS1307А: Определяется по маске NVRAM регистра 0x0F. Принудительно  
 *     сбрасывает бит CH (Clock Halt) в регистре секунд для запуска осциллятора.  
 *  3. Внутренний RTC чипа RP2040: Включается, если внешние чипы не найдены.  
 *     [Аппаратный хак]: При переходе на внутренний RTC тактирование clk_rtc переводится  
 *     на PLL_SYS, а частота clk_peri жестко фиксируется на PLL_SYS, что намертво  
 *     стабилизирует тактовую частоту АЦП и убирает разбег показаний телеметрии.  
 *  
 *  МАТРИЧНЫЙ ПЛАНИРОВЩИК (is_time_to_transmit)  
 *  -------------------------------------------  
 *  Поминутно проверяет массив beacon_schedule (до 32 задач). Поддерживает  
 *  фильтрацию по битовой маске дней недели rtc_dotw (1=Пн..7=Вс), разовые пуски  
 *  и циклические окна, в том числе со сквозным переходом через полночь  
 *  (расчет hit = (cur >= st || cur <= en)). Защищает от повторного запуска в ту же минуту.  
 *  
 *  АНТИШУМОВАЯ СТАБИЛИЗАЦИЯ АЦП ТЕЛЕМЕТРИИ  
 *  ---------------------------------------  
 *  get_telemetry_string(): Перед замером температуры кристалла RP2040  
 *  полностью останавливает аппаратную I2C шину часов (pWire->end()), переводит  
 *  пины I2C (26 и 27) в режим обычных цифровых выходов и жестко выдает на них  
 *  HIGH (+3.3 В). Это принудительно запитывает «висящий» внутренний опорный узел  
 *  АЦП, полностью убирая шумы и утечки при замере на канале 4.  
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
#include "vfo_hardware.h"

extern int pin_amp_act;          // Динамический пин активации УМ из file_manager.cpp
extern bool dev_TX_state;        // Состояние усилителя (true = передача, false = прием)

// Прямой проброс внешнего массива матричного расписания из ОЗУ
extern TaskItem beacon_schedule[MAX_SCHEDULE_TASKS];

// Глобальная частота задачи, выбранной планировщиком (0 = не задана)  
uint32_t scheduled_freq_hz = 0;   // частота из расписания

// Перечисление для типов подключенных чипов времени
RtcType activeRtc = RTC_NONE;

datetime_t currentTime;

// Имена режимов — соответствуют enum BeaconMode: 0=IFKP, 1=RTTY, 2=CW, 3=OLIVIA, 4=SEQ  
static const char* MODE_NAMES[] = {"IFKP", "RTTY", "CW", "OLIVIA", "SEQ"};  
  
// Безопасный доступ: защита от мусора в mode (например MODE_NONE=255)  
static const char* mode_name(uint8_t mode) {  
  return (mode <= MODE_SEQ) ? MODE_NAMES[mode] : "??";  
}

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

// ФИЗИЧЕСКОЕ ОБЪЯВЛЕНИЕ ПЕРЕМЕННОЙ ДЛЯ ИСПРАВЛЕНИЯ ОШИБКИ ЛИНКОВЩИКА
volatile bool tx_launching = false; 


void I2C_DS_restart() {
  // перезапуск шины Wire на линиях часов
  if (device_DS[0] == 1) {
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
  if (device_DS[0] == 1) {
    TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
    pWire->beginTransmission(RTC_I2C_ADDRESS);
    //Serial.println("[Система] пинг модуля RTC");
    return (pWire->endTransmission() == 0);
  }
return 0;
}





/**  
 * @brief Аппаратная инициализация источника времени: автоопределение RTC-чипа.  
 *  
 * Выполняет трёхконтурное определение часового чипа и запускает  
 * соответствующий источник системного времени (без внешних библиотек,  
 * прямая работа с регистрами по адресу `RTC_I2C_ADDRESS` = 0x68):  
 *  
 * -# <b>Шина.</b> @ref I2C_DS_restart переконфигурирует Wire/Wire1 на  
 *    пины из дескриптора @c device_DS (400 кГц, таймаут 50 мс).  
 *  
 * -# <b>Внешний RTC не отвечает</b> (@ref pingRTC вернул false) —  
 *    аварийный переход на внутренний RTC RP2040:  
 *    - @c activeRtc = RTC_INTERNAL, @c device_DS[0]/[4] обнуляются;  
 *    - [аппаратный хак] `clk_rtc` переводится на `PLL_SYS`  
 *      (делитель `clk_sys/46875`), а `clk_peri` жёстко фиксируется на  
 *      `PLL_SYS` — стабилизирует тактирование АЦП и убирает разбег  
 *      телеметрии; затем `adc_init()` + `rtc_init()`;  
 *    - в регистры RTC записывается стартовая дата-время по умолчанию;  
 *    - вызовом @ref update_scheduler переменные rtc_* заполняются,  
 *      время печатается в Serial и функция завершается, больше не  
 *      прикасаясь к I2C.  
 *  
 * -# <b>Внешний RTC найден</b> — автоопределение DS3231 vs DS1307A  
 *    через регистр 0x0F: записывается тестовый паттерн 0x70; если при  
 *    чтении маска 0x70 сохранилась — это DS1307A (у чипа есть  
 *    доступный NVRAM-байт), регистр затем обнуляется; иначе — DS3231.  
 *    Для DS1307A дополнительно проверяется и при необходимости  
 *    сбрасывается бит CH (Clock Halt, бит 7 регистра секунд) —  
 *    запускает остановленный осциллятор.  
 *  
 * Результат фиксируется в глобальной @c activeRtc (RTC_DS3231 /  
 * RTC_DS1307 / RTC_INTERNAL). В конце ветки внешнего чипа вызывается  
 * @ref update_scheduler и при @c debug_flag печатаются дата/время.  
 *  
 * @note Вызывается из `setup()` один раз. Источник пинов/шины —  
 *       глобальный дескриптор @c device_DS (заполняется I2C-сканером).  
 * @note При fallback на внутренний RTC время начинается с  
 *       жёстко прошитой даты (30.09.2026 15:12) — без внешнего чипа  
 *       и синхронизации календарь не устойчив к перезагрузкам.  
 *  
 * @see update_scheduler(), I2C_DS_restart(), pingRTC(),  
 *      handle_time_command(), handle_date_command(), RtcType  
 */
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
      .month = 9,
      .day   = 30,
      .dotw  = 1, 
      .hour  = 15,
      .min   = 12,
      .sec   = 00};
      
    // Безопасная аппаратная установка времени во внутренний регистр
    rtc_set_datetime(&setcurrentTime);
    delay(20);                      // ждём защёлкивания новых значений в счётчики RTC
    
    update_scheduler();

    Serial.print(get_current_time());
    Serial.print(" ");
    Serial.println(get_current_date());
    return; // МГНОВЕННЫЙ ВЫХОД, к I2C больше не прикасаемся!
  } else {
    // часы подключены
    device_DS[0] = 1;
    activeRtc = RTC_DS3231;

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
        if (debug_flag) {
          // вывод сообщения
          Serial.println("[Система] Обнаружен стандартный чип DS1307A");
        }
        pWire->beginTransmission(RTC_I2C_ADDRESS);
        pWire->write(0x0F);
        pWire->write(0x00);
        pWire->endTransmission();
      } else {
        activeRtc = RTC_DS3231;
        if (debug_flag) {
          // вывод сообщения
          Serial.println("[Система] Обнаружен чип высокой точности DS3231");
        }
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
    if (debug_flag) {
      // вывод сообщения
      char buf[34];
      snprintf(buf, sizeof(buf), " - дата      : %02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
      Serial.println(buf);
      snprintf(buf, sizeof(buf), " - время     : %02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
      Serial.println(buf);
    }
  }
}




/**  
 * @brief Обновление глобальных переменных времени и фоновых сервисов.  
 *  
 * Центральная «тиковая» функция планировщика — вызывается каждые 250 мс  
 * из `loop()` и перед каждым чтением времени из вспомогательных API  
 * (@ref get_current_time, @ref get_current_date, @ref print_current_time,  
 * @ref print_current_date).  
 *  
 * Выполняет три задачи за один проход:  
 * -# <b>Контроль пина усилителя.</b> Если @c pin_amp_act != -1, на GPIO  
 *    выводится текущее состояние @c dev_TX_state — циклическое  
 *    «страховочное» удержание ключа УМ в заданном положении.  
 * -# <b>Чтение часов.</b> При активном внешнем чипе  
 *    (@c device_DS[0] == 1 и @c activeRtc = RTC_DS3231/RTC_DS1307) шина  
 *    переинициализируется (@ref I2C_DS_restart) и из регистров 0x00-0x06  
 *    читаются 7 байт BCD: секунды (маска 0x7F — срезает бит CH), минуты,  
 *    часы (0x3F — 24-часовой формат), день недели (0x07), дата,  
 *    месяц (0x1F) и год (+2000). При внутреннем RTC RP2040 —  
 *    `rtc_get_datetime()` в @c currentTime с нормализацией дня недели  
 *    (SDK: 0=Вс → пересчёт в 1=Пн..7=Вс для единообразия с чипами).  
 * -# <b>Климат-лог.</b> Вызывается @ref climate_log_update для  
 *    периодического опроса датчиков по своим таймерам.  
 *  
 * Результат доступен через глобальные переменные: @c rtc_sec,  
 * @c rtc_min, @c rtc_hour, @c rtc_dotw, @c rtc_day, @c rtc_month,  
 * @c rtc_year — их читают @ref is_time_to_transmit,  
 * @ref get_next_start_minute и журнальные функции.  
 *  
 * @note Функция ничего не возвращает и не сообщает об ошибках чтения  
 *       I2C — при сбое шины переменные просто сохраняют прежние значения  
 *       (время «замирает» до восстановления связи).  
 * @note Вызывается с периодом 250 мс — не вызывать из прерываний и  
 *       горячих циклов: содержит I2C-транзакции и `pWire->end()`.  
 *  
 * @see init_scheduler(), I2C_DS_restart(), is_time_to_transmit(),  
 *      climate_log_update(), get_current_time(), get_current_date()  
 */
void update_scheduler() {
  // Циклический фоновый контроль состояния УМ на динамическом пине из SET.TXT
  if (pin_amp_act != -1) {
    gpio_put(pin_amp_act, dev_TX_state);
  }

  vfo_clk_thermal_guard();

  if (device_DS[0] == 1 && (activeRtc == RTC_DS3231 || activeRtc == RTC_DS1307)) {
    // если часы подключены
    I2C_DS_restart();
    TwoWire *pWire = (device_DS[1] == 1) ? &Wire1 : &Wire;
    
    pWire->beginTransmission(RTC_I2C_ADDRESS);
    pWire->write(0x00); 
    pWire->endTransmission();
    
    pWire->requestFrom(RTC_I2C_ADDRESS, (uint8_t)7); 
    if (pWire->available() >= 7) {
      rtc_sec   = bcd2bin(pWire->read() & 0x7F); // 0x00: Секунды
      rtc_min   = bcd2bin(pWire->read());        // 0x01: Минуты
      rtc_hour  = bcd2bin(pWire->read() & 0x3F); // 0x02: Часы  
      rtc_dotw  = bcd2bin(pWire->read() & 0x07); // 0x03: День недели  
      rtc_day   = bcd2bin(pWire->read());        // 0x04: Дата  
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





// печать в Serial текущих времени/даты
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


// получить стринг с текущими временем/датой
String get_current_time() {
  update_scheduler();
  char buf[16]; 
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
  return String(buf);
}

String get_current_date() {
  update_scheduler();
  char buf[16]; 
  snprintf(buf, sizeof(buf), "%02d.%02d.%04d", rtc_day, rtc_month, rtc_year);
  return String(buf);
}





/**  
 * @brief Безопасное чтение телеметрии: температура DS3231 и кристалла RP2040.  
 *  
 * Формирует строку телеметрии для передачи в эфир и вывода по командам  
 * "?" / "TELE". Выполняет два независимых замера:  
 *  
 * -# <b>Температура DS3231.</b> Только если часы определены как DS3231  
 *    (@c device_DS[0] == 1 и @c activeRtc == RTC_DS3231): шина  
 *    переинициализируется (@ref I2C_DS_restart), из регистров 0x11-0x12  
 *    читаются 2 байта температуры; значение = MSB + (LSB >> 6) * 0.25 °C.  
 *    Для DS1307 и внутреннего RTC этот блок пропускается.  
 *  
 * -# <b>Температура кристалла RP2040 (АЦП, канал 4).</b> Перед замером  
 *    аппаратная шина часов полностью останавливается (`pWire->end()`),  
 *    пины 26/27 переводятся в цифровые выходы и на них жёстко выдаётся  
 *    HIGH (+3.3 В) — это принудительно запитывает «висящий» опорный  
 *    узел АЦП и убирает утечки/шумы (см. шапку файла, раздел  
 *    «Антишумовая стабилизация АЦП»). Далее: 10 холостых чтений для  
 *    сброса заряда конденсатора выборки, затем усреднение 20 замеров.  
 *    Пересчёт: U = ADC_avg * 3.3 / 4095, T = 27 − (U − 0.706) / 0.001721.  
 *    После замера пины возвращаются шине через @ref I2C_DS_restart.  
 *  
 * @return Строка вида "T_DS =27.5C T_CPU=36.1C" при наличии DS3231  
 *         либо "T_CPU=36.1C" без него.  
 *  
 * @note Функция разрушительна для шины I2C часов на время замера —  
 *      шина останавливается и поднимается заново; вызывать только  
 *      вне передачи, когда I2C свободен.  
 * @warning Аппаратный хак с пинами 26/27 жёстко привязан к  
 *          GPIO 26/27 — не путать с динамическими пинами из SET.TXT;  
 *          если часы физически на других пинах, хак всё равно  
 *          применяется к 26/27.  
 *  
 * @see I2C_DS_restart(), update_scheduler(), get_climate_telemetry()  
 */
// Безопасное чтение телеметрии с проца и часов (при наличии температурного датчика)
String get_telemetry_string() {
  float rtc_temp = 0.0f;

  // Считываем температуру только если чип определен как DS3231
  if (device_DS[0] == 1 && activeRtc == RTC_DS3231) {
    // Передаём пины под управление I2C-контроллера Wire1/Wire
    I2C_DS_restart();
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
    snprintf(tele_buf, sizeof(tele_buf), "T_DS =%.1fC T_CPU=%.1fC", rtc_temp, mcu_temp);
  } else {
    snprintf(tele_buf, sizeof(tele_buf), "T_CPU=%.1fC", mcu_temp);
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
        delay(20);                      // ждём защёлкивания новых значений в счётчики RTC
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
        delay(20);                      // ждём защёлкивания новых значений в счётчики RTC
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
[[maybe_unused]] static bool parse_days(const String& s, uint8_t& mask) {  
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
[[maybe_unused]] static bool parse_time_expr(const String& s, TaskItem& t) {  
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



// scheduler.cpp — вывод таблицы расписания в Serial  
void print_schedule() {
  const int DAYS_WIDTH = 18;   // ширина колонки "Дни" в СИМВОЛАХ (под "Пн,Вт,Ср,Чт,Пт")  
  const int TIME_WIDTH = 18;   // ширина колонки "Время" в символах  
  
  Serial.println(F("=== РАСПИСАНИЕ ПЕРЕДАЧ ==="));
  Serial.println(F("No | МОДА | Дни               | Время             |    Частота"));
  
  uint8_t found = 0;
  for (int i = 0; i < MAX_SCHEDULE_TASKS; i++) {
    TaskItem &t = beacon_schedule[i];
    if (!t.active) continue;
    found++;
  
    // --- Дни недели: 0 = каждый день, иначе маска -> "Пн,Вт,Ср,Чт,Пт,Сб,Вс" ---  
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
    char timebuf[36];  
    if (t.interval_min == 0) {  
      snprintf(timebuf, sizeof(timebuf), "%02d:%02d", t.start_hour, t.start_min);  
    } else {  
      snprintf(timebuf, sizeof(timebuf), "%02d:%02d-%02d:%02d/%dмин",  
               t.start_hour, t.start_min, t.end_hour, t.end_min, t.interval_min);  
    }  
  
    const char* mname = mode_name(t.mode);
  
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


// scheduler.cpp — ближайший запуск для режима (MODE_CW=2? см. ниже!)  
// Возвращает абсолютные минуты от "сегодня 00:00", -1 если задач нет  
// День недели: rtc_dotw (0=Вс? уточнить конвенцию — используется та же, что в is_time_to_transmit)  
int32_t get_next_start_minute(uint8_t mode) {  
  uint32_t now_abs = rtc_hour * 60 + rtc_min;  
  int32_t best_abs = -1;  
  
  // ищем вперёд до 8 дней  
  for (uint8_t day_off = 0; day_off < 8; day_off++) {  
    uint8_t dotw = (rtc_dotw + day_off) % 7;      // конвенция та же, что у планировщика  
  
    for (int i = 0; i < MAX_SCHEDULE_TASKS; i++) {  
      TaskItem &t = beacon_schedule[i];  
      if (!t.active) continue;  
      if (t.mode != mode && t.mode != MODE_SEQ) continue; // SEQ запускает все режимы  
      if (t.days > 0 && (t.days & (1 << dotw)) == 0) continue;  
  
      uint32_t start_abs = t.start_hour * 60 + t.start_min;  
      uint32_t end_abs   = t.end_hour   * 60 + t.end_min;  
  
      if (t.interval_min == 0) {  
        // одиночная задача  
        if (day_off == 0 && start_abs <= now_abs) continue; // сегодня уже прошла  
        int32_t cand = day_off * 1440 + start_abs;  
        if (best_abs == -1 || cand < best_abs) best_abs = cand;  
      } else {  
        // периодическая: первый шаг после now (для сегодня), либо сам старт (для будущих дней)  
        uint32_t first = (day_off == 0)  
          ? ((now_abs > start_abs) ? start_abs + ((now_abs - start_abs + t.interval_min - 1) / t.interval_min) * t.interval_min  
                                   : start_abs)  
          : start_abs;  
        if (first <= end_abs) {  
          int32_t cand = day_off * 1440 + first;  
          if (best_abs == -1 || cand < best_abs) best_abs = cand;  
        }  
      }  
    }  
    if (best_abs != -1) break; // нашли в ближайший подходящий день — дальше не ищем  
  }  
  return best_abs;  
}



/**  
 * @brief Форматирует абсолютное время ближайшего старта в строку "HH:MM".  
 *  
 * Преобразует результат @ref get_next_start_minute (абсолютное число минут  
 * от текущих суток, может превышать 1440 — т.е. старт в следующих днях)  
 * в человекочитаемую строку вида "HH:MM". Для старта не сегодня добавляет  
 * суффикс: " (завтра)" при day_off == 1 и " (через дни)" при day_off >= 2.  
 *  
 * @param[in] abs_min  Абсолютное время старта в минутах от полуночи текущего  
 *                     дня (может быть > 1440). Отрицательное значение  
 *                     означает отсутствие подходящей задачи.  
 *  
 * @return Строка формата "HH:MM", "HH:MM (завтра)", "HH:MM (через дни)"  
 *         либо "Не задан", если @p abs_min < 0.  
 *  
 * @note Функция чистая: не читает RTC и не изменяет глобальное состояние.  
 *       Используется для отображения расписания в print_schedule() и веб/UI.  
 *  
 * @see get_next_start_minute(), print_schedule()  
 */
String fmt_next_start(int32_t abs_min) {
// преобразование результата в строку "HH:MM" или "завтра HH:MM"
  if (abs_min < 0) return String("Не задан");  
  uint32_t day_off = abs_min / 1440;  
  uint32_t m = abs_min % 1440;  
  char buf[48];
  const char* suffix = (day_off == 0) ? "" : (day_off == 1 ? " (завтра)" : " (через дни)");  
  snprintf(buf, sizeof(buf), "%02d:%02d%s", (int)(m / 60), (int)(m % 60), suffix);  
  return String(buf);   
}  
  



/**  
 * @brief Проверяет, наступило ли время запуска передачи для заданного режима.  
 *  
 * Перебирает задачи расписания @ref beacon_schedule и проверяет, попадает ли  
 * текущее время RTC (минуты от полуночи, @ref rtc_hour / @ref rtc_min) в окно  
 * хотя бы одной активной задачи указанного режима @p mode.  
 *  
 * Поддерживаются два типа задач:  
 * - разовый запуск (@c interval_min == 0): срабатывание точно в минуту старта;  
 * - периодический запуск (@c interval_min > 0): срабатывание каждые  
 *   @c interval_min минут внутри окна @c start..@c end, включая окно,  
 *   переходящее через полночь.  
 *  
 * Дополнительно фильтруются задачи по дню недели (@c TaskItem::days)  
 * и флагу активности (@c TaskItem::active). Повторный запуск одного режима  
 * в пределах той же минуты блокируется статическим массивом @c last_min.  
 *  
 * При успешном совпадении функция записывает частоту задачи в  
 * @ref scheduled_freq_hz и выводит диагностику в Serial. Если в момент  
 * срабатывания установлен флаг @c is_transmitting, сеанс пропускается.  
 *  
 * @param[in] mode  Режим передачи: MODE_IFKP (0), MODE_RTTY (1), MODE_CW (2), MODE_OLIVIA (3),
 *                  MODE_SEQ (4). Значения > 4 отклоняются.  
 *  
 * @return @c true  — задача совпала, передачу нужно запустить  
 *                   (вызывающий код обязан выставить @c is_transmitting);  
 * @return @c false — ни одна задача не совпала, либо шина занята,  
 *                   либо сеанс этого режима уже был в текущей минуте.  
 *  
 * @note Сама функция не выполняет передачу — только детектирует момент  
 *       и подготавливает @ref scheduled_freq_hz. Флаг @c is_transmitting  
 *       выставляется вызывающим кодом после возврата @c true.  
 *  
 * @warning При пропуске сеанса из-за занятой шины @c last_min не  
 *          обновляется — возможны повторные лог-сообщения и запоздалый  
 *          запуск в той же минуте после освобождения эфира.  
 *  
 * @see get_next_start_minute(), TaskItem, beacon_schedule  
 */
bool is_time_to_transmit(uint8_t mode) {  
  extern bool is_transmitting;  
  extern volatile bool tx_launching;   // Объявлено как extern для явного связывания
                                       // флаг "передача запускается" — выставляется  
                                       // вызывающим кодом СРАЗУ при получении true  
  uint32_t cur = rtc_hour * 60UL + rtc_min;  
  static uint32_t last_min[4] = {9999, 9999, 9999, 9999};  
  
  if (mode > 4) return false;  
  if (cur == last_min[mode]) return false;   // один запуск (или пропуск) на минуту  
  
  // Проверка занятости ДО перебора задач — закрывает окно гонки  
  // между опросами разных режимов и исключает спам лога при hit.  
  if (is_transmitting || tx_launching) {  
    // Ничего не логируем здесь — задачи ещё не проверены,  
    // лог ниже сообщит, была ли реальная сработка.  
  }  
  
  for (int i = 0; i < MAX_SCHEDULE_TASKS; i++) {  
    TaskItem& t = beacon_schedule[i];  
    if (!t.active || t.mode != mode) continue;  
    if (t.days && !(t.days & (1 << rtc_dotw))) continue;   // день не совпал  
  
    uint32_t st = t.start_hour * 60UL + t.start_min;  
    uint32_t en = t.end_hour   * 60UL + t.end_min;  
    bool hit;  
    if (t.interval_min == 0) hit = (cur == st);  
    else {  
      hit = (en >= st) ? (cur >= st && cur <= en && (cur - st) % t.interval_min == 0)  
                       : (cur >= st || cur <= en) &&        // окно через полночь  
                         ((cur >= st ? cur - st : cur + 1440 - st) % t.interval_min == 0);  
    }  
    if (!hit) continue;  

    // Ветка пропуска при занятости устройства или предпусковом состоянии
    if (is_transmitting || tx_launching) {  
      Serial.printf("[Планировщик] Задача %02d: устройство занято, сеанс пропущен\n", i + 1);  
      last_min[mode] = cur;   // фиксируем пропуск: без спама и без запоздалого  
      return false;           // запуска в той же минуте после освобождения шины  
    }  
  
    Serial.printf("[Планировщик] Задача %02d: запуск %s в %02d:%02d\n",  
                  i + 1, mode_name(mode), rtc_hour, rtc_min);  
    last_min[mode] = cur;  
    scheduled_freq_hz = beacon_schedule[i].freq_hz;   // частота из задачи  
    tx_launching = true;  // Атомарно занимаем шину перед выходом из планировщика
    return true;  
  }  
  return false;  
}


