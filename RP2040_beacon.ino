/**
 * ============================================================================  
 *  beacon_v2_10.ino — Автоматический радиомаяк на RP2040 (главный скетч)  
 *  Версия 2.10.3 от 2026-09-29, автор RU0AOG  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Ядро прошивки автоматического КВ/УКВ-радиомаяка: связывает все модули  
 *  (генератор, модуляторы, часы, датчики, дисплей, индикацию, файловую систему)  
 *  и по расписанию выводит в эфир сообщения тремя режимами — IFKP, RTTY, CW.  
 *  ВЧ-сигнал формируется либо внешним синтезатором Si5351, либо аварийным  
 *  программным DDS-генератором на PIO RP2040 (vfo_hardware) при отсутствии чипа.  
 *  
 *  ГЛАВНЫЙ ЦИКЛ (loop)  
 *  -------------------  
 *  1. Обработка «мягкого рестарта» (soft_restart_flag) — перечитывание  
 *     конфигурации без перепрошивки.  
 *  2. Системный такт 250 мс -> update_scheduler() (обновление времени RTC).  
 *  3. Раз в 5 секунд — вывод телеметрии на LCD и мигание индикатором.  
 *  4. Опрос USB-диска (check_and_handle_pc_changes) и консоли  
 *     (check_serial_commands).  
 *  5. Три независимых блока сеансов: IFKP (режим 0), RTTY (режим 1), CW  
 *     (режим 2) — запускаются по расписанию (is_time_to_transmit) либо  
 *     принудительно флагами force_*_transmission.  
 *  
 *  СТРУКТУРА СЕАНСА ПЕРЕДАЧИ (одинакова для всех режимов)  
 *  -----------------------------------------------------  
 *  SI_POWER_ON -> подготовка частот -> позывной/локатор -> текст ->  
 *  телеметрия (системная + климатическая) -> "OVER" -> SI_POWER_OFF.  
 *  Каждый шаг обёрнут проверкой (!pc_file_written && !soft_restart_flag) —  
 *  это механизм экстренного прерывания эфира от ПК или команды STOP.  
 *  
 *  АВТООПРЕДЕЛЕНИЕ ОБОРУДОВАНИЯ (I2C_Scanner / scanRP2040Ports)  
 *  -----------------------------------------------------------  
 *  При старте сканируются обе шины I2C (i2c0 и i2c1) по всем допустимым парам  
 *  пинов SDA/SCL с исключением GPIO16/23/24/25 (RGB-светодиод и отсутствующие  
 *  пины). Найденные приборы (Si5351 0x60, DS3231/DS1307 0x68, AT24Cxx 0x50-0x57,  
 *  LCD 0x27, BME/BMP280 0x76, BMP180 0x77) заносятся в таблицы device_*[5]  
 *  (формат: [активен, № шины, SDA, SCL, адрес]). Тип RTC и объём EEPROM  
 *  определяются активными тестами записи/чтения регистров.  
 *  
 *  КОНСОЛЬНЫЕ КОМАНДЫ (check_serial_commands)  
 *  ------------------------------------------  
 *    ? / tele / time / date / text / setparam M=V /  
 *    start cw|rtty|ifkp / stop / restart / reset / help.  
 *  
 *  ГЛОБАЛЬНОЕ СОСТОЯНИЕ:  
 *    device_SI/DS/BM/AT/DL[5]  — таблицы найденного оборудования.  
 *    soft_restart_flag, pc_file_written, is_transmitting — флаги управления.  
 *    BCN_VER / BCN_DAT          — версия и дата прошивки.  
 * ============================================================================  
 */

#include "file_manager.h"
#include "si5351_driver.h"
#include "ifkp_modem.h"
#include "rtty_modem.h"  
#include "scheduler.h"
#include "cw_modem.h"
#include "bme280.h"
#include "lcd.h"
#include "led_blink.h"
#include "climate_log.h"
#include <hardware/watchdog.h>
#include <hardware/adc.h>
#include "vfo_hardware.h"
#include <Adafruit_TinyUSB.h>

String BCN_VER = "2.11";
String BCN_DAT = "2026-09-30";

bool dev_TX_state  = false;

// таблица параметров устройств
/*
  device_BM[0] = 1;
  device_BM[1] = WIRE_NO;
  device_BM[2] = PIN_SDA;
  device_BM[3] = PIN_SCL;
  device_BM[4] = address;  */
uint8_t device_SI[5];
uint8_t device_DS[5];
uint8_t device_BM[5];
uint8_t device_AT[5];
uint8_t device_DL[5];

String device_SI_name = "SI5351 Генератор";
String device_DS_name = "DS3231 Часы RTC";
String device_BM_name = "BME280 Барометр";
String device_AT_name = "EEPROM AT24C";
String device_DL_name = "дисплей LCD1602/1604";

bool force_cw_transmission   = false; // Флаг ручного запуска CW
bool force_rtty_transmission = false; // Флаг ручного запуска RTTY
bool force_ifkp_transmission = false; // Флаг ручного запуска IFKP

// --- НАСТРОЙКИ АВТОМАТА КНОПКИ USR ---
const uint8_t PIN_USR_BUTTON = 24; // Системный пин кнопки BOOT/USR на большинстве плат RP2040

enum UsrChainState {
  USR_IDLE,          // Ожидание нажатия кнопки
  USR_START_CW,      // Запуск и выполнение сеанса CW
  USR_WAIT_FOR_RTTY, // 15-секундная пауза перед RTTY
  USR_START_RTTY,    // Запуск и выполнение сеанса RTTY
  USR_WAIT_FOR_IFKP, // 15-секундная пауза перед IFKP
  USR_START_IFKP,    // Запуск и выполнение сеанса IFKP
  USR_CLOSE_CHAIN    // Финальная стадия освобождения шины после IFKP
};

UsrChainState usr_chain_state = USR_IDLE;  // Текущий статус автомата последовательности
uint32_t usr_timer_ms = 0;                 // Таймер неблокирующей паузы
bool old_button_state = HIGH;              // Предыдущее состояние кнопки для отслеживания клика

bool soft_restart_flag = false;
static bool wd_reboot_detected = false;
extern bool pc_activity_detected; 
bool is_transmitting = false; // Флаг передачи 
String rtc_chip_name = "Неизвестный RTC"; // Сюда сканер запишет точное имя чипа

extern uint8_t rtc_sec;

// Функция проверки и обработки текстовых команд с локальным эхом
extern void update_info_config_from_console(String marker, String new_value);

void check_serial_commands() {  
  static char cmd_buffer[64];  
  static size_t buf_idx = 0;  
  static uint32_t last_input_ms = 0;   // время последнего принятого символа  
  static bool reminder_shown = false;  // защёлка, чтобы напоминание не повторялось

  // Цикл работает, пока не вычитает ВСЕ доступные символы из UART
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    last_input_ms = millis();  
    reminder_shown = false;   // новый ввод — сбрасываем защёлку

    if (c == '\n' || c == '\r') {
      if (buf_idx > 0) {
        cmd_buffer[buf_idx] = '\0'; 
        String command = String(cmd_buffer); 
        // Сразу сбрасываем буфер, так как команда скопирована в String
        buf_idx = 0; 

        command.trim(); 

        if (command.length() > 0) {
          // перевести строку и вывести введённую команду
          Serial.print(F("\r\n> "));
          Serial.println(command);
          
          LCD_print("                    ", 1, 0);
          LCD_print(">"+command, 1, 0);

          if (command.equalsIgnoreCase("?")) {
            print_current_settings();
            print_current_date();
            print_current_time();
            Serial.print(F("Телеметрия: ")); Serial.println(get_telemetry_string());
            if (device_BM[0] == 1) {
            Serial.print(F("Телеметрия: ")); Serial.println(get_climate_telemetry());
            }
            else {
            Serial.print(F("Телеметрия: ")); Serial.println("в системе отсутствует датчик давления");
            }
            Serial.println(F("Введите help для перехода в справочное меню по командам управления"));
          }
          else if (command.equalsIgnoreCase("TELE")) {
            print_current_time();
            Serial.print(F("Телеметрия: ")); Serial.println(get_telemetry_string());
            if (device_BM[0] == 1) {
            Serial.print(F("Телеметрия: ")); Serial.println(get_climate_telemetry());
            }
            else {
            Serial.print(F("Телеметрия: ")); Serial.println("в системе отсутствует датчик давления");
            }
          }
          else if (command.startsWith("time")) {
            int space_idx = command.indexOf(' ');
            if (space_idx == -1) {
              print_current_date();
              print_current_time();
            } else {
              handle_time_command(command);
            }
          }
          else if (command.startsWith("date")) {
            handle_date_command(command);
          }
          else if (command.equalsIgnoreCase("RESTART") || command.equalsIgnoreCase("STOP")) {
            if (command.equalsIgnoreCase("RESTART")) {
              Serial.println(F("[!] МЯГКИЙ ПЕРЕЗАПУСК МАЯКА..."));
              pc_file_written = true;
              LCD_init(true);
              LCD_print(">> RESTART <<", 0, 0);
            } else {
              Serial.println(F("[!] Экстренная остановка передачи"));
              LCD_print("STOP TRANSMIT!!!", 0, 0);
            }
            Serial.flush();
            soft_restart_flag = true;
          }
          else if (command.equalsIgnoreCase("RESET")) {
            Serial.println(F("[!] КРИТИЧЕСКИЙ ЖЕСТКИЙ СБРОС ПРОЦЕССОРА..."));
            LCD_init(true);
            LCD_print(">> CPU RESET <<", 0, 0);
            // Пишем в файл на виртуальную флешку историю работы
            log_file_write_line("рестарт процессора по команде оператора");
            SI_POWER_OFF();
            Serial.println(F("***"));
            Serial.println(F(""));
            Serial.flush();
            delay(500);
            watchdog_reboot(0, 0, 0);
          }
          else if (command.startsWith("text")) {
            int space_idx = command.indexOf(' ');
            if (space_idx == -1) {
              Serial.println(F("=== ТЕКУЩИЙ ТЕКСТ РАДИОПЕРЕДАЧИ МАЯКА ==="));
              Serial.print(F("Текст: ")); 
              Serial.println(my_text_variable.length() > 0 ? my_text_variable : F("Не задан"));
              Serial.println(F("========================================="));
            } else {
              String new_txt = command.substring(space_idx + 1);
              new_txt.trim();
              if (new_txt.length() > 0) {
                update_info_config_from_console("TEXT", new_txt);
              } else {
                Serial.println(F("[Ошибка] Текст для записи пустой."));
              }
            }
          }
          else if (command.equalsIgnoreCase("START CW")) {
            if (is_transmitting) {
              Serial.println(F("[Ошибка] Сейчас уже идет трансляция!"));
            } else {
              Serial.println(F("[Система] Заявка принята. Выходим в эфир CW..."));
              force_cw_transmission = true;
            }
          }
          else if (command.equalsIgnoreCase("START RTTY")) {
            if (is_transmitting) {
              Serial.println(F("[Ошибка] Сейчас уже идет трансляция!"));
            } else {
              Serial.println(F("[Система] Заявка принята. Выходим в эфир RTTY..."));
              force_rtty_transmission = true;
            }
          }
          else if (command.equalsIgnoreCase("START IFKP")) {
            if (is_transmitting) {
              Serial.println(F("[Ошибка] Сейчас уже идет трансляция!"));
            } else {
              Serial.println(F("[Система] Заявка принята. Выходим в эфир IFKP..."));
              force_ifkp_transmission = true;
            }
          }
          // --- НОВАЯ КОРРЕКТИРОВКА: ОЧИСТКА ЖУРНАЛА LOG.TXT ИЗ КОНСОЛИ ---
          else if (command.equalsIgnoreCase("clear log")) {
            if (is_transmitting) {
              Serial.println(F("[Ошибка] Нельзя стирать журнал во время активной передачи в эфир!"));
            } else {
              Serial.println(F("[Система] Запрос принят. Стираем LOG.TXT..."));
              LCD_init(true);
              LCD_print("CLEARING LOG...", 0, 0);
              
              log_file_clear(); // Вызов нашей новой функции очистки ОЗУ и Flash
              
              LCD_print("LOG FILE BLANK", 0, 0);
              Serial.println(F("[Система] Журнал успешно очищен!"));
              // Пишем в файл на виртуальную флешку историю работы
              log_file_write_line("Лог очищен");
            }
          }
          // ПРИНУДИТЕЛЬНЫЙ СБРОС ВСЕХ ФАЙЛОВ НА ДЕФОЛТ ---
          else if (command.equalsIgnoreCase("format disk")) {
            if (is_transmitting) {
              Serial.println(F("[Ошибка] Нельзя форматировать диск во время активной передачи в эфир!"));
            } else {
              LCD_init(true);
              LCD_print("FORMATTING DISK.", 0, 0);
              
              force_reset_to_default_disk(); // Вызов новой функции сброса
              
              LCD_print("DISK DEFAULT OK", 0, 0);
              // Записываем событие в свежесозданный дефолтный лог
              log_file_write_line("флэш-диск отформатирован. Все установки сброшены");
            }
          }
          else if (command.equalsIgnoreCase("sched") || command.equalsIgnoreCase("schedule")) {  
            print_schedule();  
          }
          else if (command.startsWith("setparam ")) {
            String param_part = command.substring(9);
            param_part.trim();
            int eq_idx = param_part.indexOf('=');
            if (eq_idx != -1) {
              String marker = param_part.substring(0, eq_idx);
              String value  = param_part.substring(eq_idx + 1);
              marker.trim();
              value.trim();
              if (marker.length() > 0 && value.length() > 0) {
                update_info_config_from_console(marker, value);
              }
            } else {
              Serial.println(F("[Ошибка] Неверный формат. Используйте: setparam МАРКЕР=ЗНАЧЕНИЕ"));
            }
          }
          else {
          Serial.println(F("=== СПРАВКА ПО КОМАНДАМ УПРАВЛЕНИЯ ==="));
          Serial.println(F("help(или другое)- Вывести это справочное меню"));
          Serial.println(F("?               - Показать текущую конфигурацию из INFO.txt, дату и время RTC"));
          Serial.println(F("tele            - Вывести данные телеметрии"));
          Serial.println(F("time            - Вывести текущее время и дату"));
          Serial.println(F("time ЧЧ:ММ      - Установить время часов, например: time 12:45"));
          Serial.println(F("time ЧЧ:ММ:CC   - Установить время часов, например: time 19:30:0)"));
          Serial.println(F("date ДД.ММ.ГГГГ - Установить календарную дату, например: date 18.05.2026"));
          Serial.println(F("text            - Показать текущий текст радиопередачи из конфигурации INFO.txt"));
          Serial.println(F("text [текст]    - Записать новый текст передачи в файл конфигурации, например: text NEW TEXT"));
          Serial.println(F("setparam M=V    - Изменить любой маркер в конфигурации, например: setparam CALL=RA3ABC или setparam QTH=NA56AV"));
          Serial.println(F("sched           - Показать таблицу расписания передач из INFO.txt"));
          Serial.println(F("start cw        - Немедленно запустить внеочередной сеанс CW"));
          Serial.println(F("start rtty      - Немедленно запустить внеочередной сеанс RTTY"));
          Serial.println(F("start ifkp      - Немедленно запустить внеочередной сеанс IFKP"));
          Serial.println(F("clear log       - Стереть существующий файл LOG.TXT и создать новый пустой"));
          Serial.println(F("format disk     - Полностью стереть диск и записать дефолтные INFO.TXT, SET.TXT, LOG.TXT"));
          Serial.println(F("stop            - Экстренная остановка передачи маяка"));
          Serial.println(F("restart         - Мягкий виртуальный перезапуск маяка"));
          Serial.println(F("reset           - Жесткий аппаратный сброс процессора RP2040"));
          Serial.println(F("======================================="));
          }
        }
      }
    } 
    else {
      if (buf_idx < sizeof(cmd_buffer) - 1) {
          cmd_buffer[buf_idx++] = c;
      } else {
          buf_idx = 0; // Защита от переполнения
          reminder_shown = true;  
          Serial.println(F("\n[Ошибка] Команда длиннее 63 символов — буфер очищен."));  
      }
    }
  } // Конец while
  
  // Напоминание: буфер не пуст, а терминатор не приходит уже 10 секунд  
  if (!reminder_shown && buf_idx > 0 && last_input_ms != 0 &&  
      (millis() - last_input_ms > 10000)) {  
    reminder_shown = true;  
    Serial.println(F("\n[Подсказка] Команда набрана, но не отправлена. "  
                     "Включите в терминале перевод строки (NL или NL&CR)."));  
    Serial.print(F("> "));  
    Serial.write(cmd_buffer, buf_idx);   // выводит ровно buf_idx байт, '\0' не нужен  
    Serial.println();  
  }  
}




void setup() {
  adc_init();
  init_usb_msc_interface();                       // Регистрирует дескрипторы Mass Storage в стек TinyUSB до начала энумерации хостом
  init_file_manager();

  pinMode(PIN_USR_BUTTON, INPUT_PULLUP);          // инициализируем пин кнопки
  delay(20);                                      // Даем Pull-up надежно поднять линию до +3.3 В
  old_button_state = digitalRead(PIN_USR_BUTTON); // Фиксируем РЕАЛЬНОЕ стартовое состояние (HIGH)

  wd_reboot_detected = watchdog_caused_reboot(); 

  // запуск виртуального СОМ-порта через USB
  Serial.begin(115200);

  // Serial1.begin(115200, SERIAL_8N1); // Запуск аппаратного UART0 на пинах GP0 (TX) и GP1 (RX)
  // Serial.println("[Система] Выходим в эфир...");  // это пойдет в USB
  // Serial1.println("[Система] Выходим в эфир..."); // это пойдет по физическому проводу TX

  // ожидание коннекта - до 5 сек
  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 5000)) {
      #if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
      TinyUSBDevice.task();
      #endif
  }
  float exactSeconds = (millis() - timeout) / 1000.0;
  Serial.print("старт порта за ");
  Serial.print(exactSeconds, 2); 
  Serial.println(" сек");
  //Serial.println("...");

  Serial.println("[Система] Питание маяка включено.");
  //Serial.println("[Система] Последовательное соединение восстановлено.");
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("[Система] "); Serial.println(my_FAT);


  I2C_Scanner();       // сканировать шину I2C, формировать таблицу устройств

  init_BME();          // инициализировать bme280

  init_scheduler();    // инициализировать дс3231

  init_si5351();       // инициализировать си5351

  ZERO_LED_init();     // инициализировать WS2812B

  init_flash_disk();   // инициализировать флэш-диск для Windows


  // Инициализация пинов питания шин, если они назначены (не равны -1)
  if (pin_pwr_si != -1) { pinMode(pin_pwr_si, OUTPUT); digitalWrite(pin_pwr_si, LOW); }
  if (pin_pwr_ds != -1) { pinMode(pin_pwr_ds, OUTPUT); digitalWrite(pin_pwr_ds, HIGH); } // Часы обычно всегда запитаны
  if (pin_pwr_bm != -1) { pinMode(pin_pwr_bm, OUTPUT); digitalWrite(pin_pwr_bm, LOW); }
  if (pin_pwr_dl != -1) { pinMode(pin_pwr_dl, OUTPUT); digitalWrite(pin_pwr_dl, LOW); }
  // ПРАВКА: Инициализация динамического пина активации УМ из файла SET.TXT
  if (pin_amp_act != -1) {
    pinMode(pin_amp_act, OUTPUT);
    digitalWrite(pin_amp_act, dev_TX_state);
  }

  watchdog_enable(15000, true);   // 15 сек; true = не тикать при остановке по отладчику
  if (wd_reboot_detected) {  
    Serial.println(F("[Watchdog] Обнаружен перезапуск по сторожевому таймеру!"));  
    log_file_write_line("Watchdog: аварийный перезапуск");   // попадёт в LOG.TXT с датой/временем RTC  
  }
  else Serial.println(F("[Система] Запуск сторожевого таймера - ок"));

  //Serial.println(F("\n================================================================"));
  Serial.println(F("  АВТОМАТИЧЕСКИЙ РАДИОМАЯК ЗАПУЩЕН"));
  Serial.print(F("  версия "));
  Serial.print(BCN_VER);
  Serial.print(F(" прошивка от "));
  Serial.println(BCN_DAT);
  Serial.println(F("  Плата готова к работе."));
  Serial.print(F("  Текущая дата ")); Serial.print(get_current_date()); Serial.print(F(" время ")); Serial.println(get_current_time());
  Serial.println(F("  Введите команду или h для выхода в справочное меню."));
  Serial.println(F("=========================================================================="));

}

void loop() {
  #if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
  TinyUSBDevice.task(); 
  #endif
  
  watchdog_update();  // сброс сторожевого таймера
  
  extern uint8_t rtc_sec; 
  // =========================================================================
  // 1. ОБРАБОТКА МЯГКОГО РЕСТАРТА
  // =========================================================================
  if (soft_restart_flag) {
    soft_restart_flag = false; 
    current_tone = 0;
    usr_chain_state = USR_IDLE;
    delay(100);
    read_file_to_variable(); // Извлекаем чистые маркеры
    pc_file_written = false;
    Serial.println("Рестарт: OK");
    //print_current_settings();
    return; // Сразу уходим на новый виток loop, сбрасывая старые флаги
  }



  // =========================================================================
  // СИСТЕМНЫЙ АВТОМАТ ПОСЛЕДОВАТЕЛЬНОЙ ПЕРЕДАЧИ ПО КНОПКЕ USR (С БЛОКИРОВКОЙ)
  // =========================================================================
  bool current_button_state = digitalRead(PIN_USR_BUTTON);
  
  // Отслеживаем клик (переход из HIGH в LOW). 
  // Запуск возможен только из режима ожидания (USR_IDLE) и если станция абсолютно свободна (!is_transmitting)
  if (current_button_state == LOW && old_button_state == HIGH && !is_transmitting && usr_chain_state == USR_IDLE && millis() > 1000) {
    Serial.println(F("[Кнопка]  Нажата кнопка USR! Запуск цепочки с блокировкой планировщика..."));
    LCD_init(true);
    LCD_print("USR CHAIN ACTIVE", 0, 0);
    is_transmitting = true;         // Включаем глобальную блокировку планировщика
    usr_chain_state = USR_START_CW; // Переводим автомат на первый шаг
  }
  old_button_state = current_button_state;

  // Отработка шагов автомата
  switch (usr_chain_state) {
    case USR_START_CW:
      // Передаем управление модулю CW. Флаг is_transmitting уже равен true, планировщик заблокирован
      Serial.println(F("[Автомат] Шаг 1: Запуск внеочередной передачи CW."));
      force_cw_transmission = true; 
      usr_chain_state = USR_WAIT_FOR_RTTY;
      break;

    case USR_WAIT_FOR_RTTY:
      // Ждем, пока внутренний модулятор CW отработает.
      // В конце своего сеанса модулятор CW сбросит force_cw_transmission в false.
      // ВНИМАНИЕ: Родной блок CW в конце сеанса выполнит "is_transmitting = false". 
      // Мы перехватываем этот момент, возвращаем true для удержания блокировки во время паузы.
      if (!force_cw_transmission && !is_transmitting) {
        Serial.println(F("[Автомат] Передача CW завершена. Блокировка удержана. Пауза 15 сек перед RTTY..."));
        is_transmitting = true;   // Восстанавливаем блокировку на время тишины!
        usr_timer_ms = millis();  // Засекаем 15 секунд паузы между модами
        usr_chain_state = USR_START_RTTY;
      }
      break;

    case USR_START_RTTY:
      // Удерживаем блокировку, пока тикают 15 секунд
      if (millis() - usr_timer_ms >= 15000) {
        if (pc_file_written || soft_restart_flag) { is_transmitting = false; usr_chain_state = USR_IDLE; break; }
        Serial.println(F("[Автомат] Шаг 2: Время вышло. Запуск передачи RTTY."));
        force_rtty_transmission = true; 
        usr_chain_state = USR_WAIT_FOR_IFKP;
      } else {
        is_transmitting = true;   // Защитное удержание флага занятости во время тиканья таймера
      }
      break;

    case USR_WAIT_FOR_IFKP:
      // Ждем окончания сеанса RTTY (когда модулятор RTTY сбросит свой force-флаг)
      if (!force_rtty_transmission && !is_transmitting) {
        Serial.println(F("[Автомат] Передача RTTY завершена. Блокировка удержана. Пауза 15 сек перед IFKP..."));
        is_transmitting = true;   // Снова принудительно держим флаг занятости передатчика
        usr_timer_ms = millis();  // Засекаем вторые 15 секунд тишины
        usr_chain_state = USR_START_IFKP;
      }
      break;

    case USR_START_IFKP:
      // Удерживаем блокировку во время второй паузы
      if (millis() - usr_timer_ms >= 15000) {
        if (pc_file_written || soft_restart_flag) { is_transmitting = false; usr_chain_state = USR_IDLE; break; }
        Serial.println(F("[Автомат] Шаг 3: Время вышло. Запуск передачи IFKP."));
        force_ifkp_transmission = true; 
        usr_chain_state = USR_CLOSE_CHAIN; // Переходим к финальной стадии освобождения шины
      } else {
        is_transmitting = true;
      }
      break;

    case USR_CLOSE_CHAIN:
      // Финальный шаг: ждем, пока отработает IFKP модулятор
      if (!force_ifkp_transmission && !is_transmitting) {
        Serial.println(F("[Автомат] Передача IFKP завершена. Вся цепочка выполнена успешно. Снимаем блокировку."));
        LCD_init(true);
        LCD_print("CHAIN DONE! READY", 0, 0);
        usr_chain_state = USR_IDLE; // Полное обнуление автомата, is_transmitting теперь честно равен false
      }
      break;

    case USR_IDLE:
    default:
      break;
  }

  // Аварийный останов всей ручной цепочки и сброс блокировок, если ПК подключился к диску
  if (pc_file_written) {
    is_transmitting = false;
    usr_chain_state = USR_IDLE;
  }


  // =========================================================================
  // 2. СИСТЕМНЫЙ ТАКТ И ПЛАНИРОВЩИК (250 мс)
  // =========================================================================
  static uint32_t last_rtc_tick = 0;
  if (millis() - last_rtc_tick >= 250) {
    last_rtc_tick = millis();
    update_scheduler();
  }

  // =========================================================================
  // 3. ПЕРИОДИЧЕСКИЙ ВЫВОД ИНФОРМАЦИИ (Каждые 5 секунд)
  // =========================================================================
  static uint8_t last_printed_sec = 255; // Статическая переменная для хранения секунды

  // Проверяем, кратна ли текущая секунда 5
  if (rtc_sec % 5 == 0 && rtc_sec != last_printed_sec) {
    // Проверяем условия: отсутствие эфира и активности
    if (!pc_activity_detected && !soft_restart_flag && !is_transmitting) {
      last_printed_sec = rtc_sec; // Запоминаем секунду, блокируя повторные вызовы внутри нее
      
      update_scheduler();
      //char buf[34];
      //snprintf(buf, sizeof(buf), "[Таймер] %02d:%02d:%02d", rtc_hour, rtc_min, rtc_sec);
      //Serial.print(buf);
      //Serial.print(F(" ")); Serial.print(get_telemetry_string());
      //Serial.print(F(" ")); Serial.println(get_climate_telemetry());
      ZERO_LED_GREEN_ON();
      delay(5);
      ZERO_LED_OFF();

      LCD_init(true);   
      String T_CPU_text = "";
      String T_DS_text  = "T1=" + get_telemetry_string().substring(5, 9);
      if (device_DS[0] == 1 && activeRtc == RTC_DS1307) {T_CPU_text = "CPU " + get_telemetry_string().substring(15, 19) + " ";}
      if (device_DS[0] == 1 && activeRtc == RTC_DS3231) {T_CPU_text = "CPU " + get_telemetry_string().substring(17, 21) + " ";}
      if (device_DS[0] == 0) {                           T_CPU_text = "CPU " + get_telemetry_string().substring(15, 19) + " ";}
      String T_CL_text  = "TMP " + get_climate_telemetry().substring(5, 9) + " ";
      String P_CL_text  = get_climate_telemetry().substring(17, 22) + "mm";
      String Time_text  = "  " + get_current_time().substring(0, 5);

      LCD_print(T_CL_text, 1, 0);
      LCD_print(P_CL_text, 1, 9);
      LCD_print(T_CPU_text, 0, 0);
      LCD_print(Time_text, 0, 9);
    }
  }

  

  // =========================================================================
  // 4. ОПРОС ИНТЕРФЕЙСОВ И КОМАНД
  // =========================================================================
  check_and_handle_pc_changes();
  check_serial_commands();

  // Если за время проверки команд взлетел флаг рестарта — выходим
  if (soft_restart_flag || pc_file_written) return;

  // =========================================================================
  // РЕЖИМ 0. СЕАНС СВЯЗИ IFKP
  // =========================================================================
  if (( (is_time_to_transmit(0) && !is_transmitting) || force_ifkp_transmission) && !pc_file_written && !soft_restart_flag) {
    force_ifkp_transmission = false;
    is_transmitting = true;
    
    char time_buf[128];
    snprintf(time_buf, sizeof(time_buf), "[Система] %02d:%02d:%02d - Наступило время сеанса IFKP! Выходим в эфир...", rtc_hour, rtc_min, rtc_sec);
    Serial.println(time_buf);
    
    if ((my_call_variable.length() > 0 || my_text_variable.length() > 0) && !soft_restart_flag) {
      
      // Засекаем точное время старта сеанса связи IFKP
      uint32_t ifkp_session_start_ms = millis();
      dev_TX_state = true;
      update_scheduler();
      SI_POWER_ON();
      
      if (!pc_file_written && !soft_restart_flag) {
        uint32_t ifkp_hz = strtoul(my_freq_ifkp_var.c_str(), NULL, 10);
        if (ifkp_hz == 0) ifkp_hz = 3601307; 
        
        // Вывод параметров IFKP
        prepare_ifkp_frequencies(ifkp_hz);
        
        // IFKP: передача стартовой калибровочной лесенки (33 тона)
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_IFKP] Калибровочная лесенка "));
          // Вызываем готовую безопасную функцию из модема
          send_ifkp_calibration_ladder(); 
          Serial.println(F("ОК"));
        }
        
        // IFKP: передача позывного и локатора
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_IFKP] Вызов: "));
          send_ifkp_string("\r\n\r\nVVV BEACON DE " + my_call_variable + "/B DE " + my_call_variable + "/B, QTH " + my_qth_variable + " " + my_qth_variable + " \r\n"); 
        }
        
        // IFKP: передача стартовой калибровочной лесенки (33 тона)
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_IFKP] Калибровочная лесенка "));
          // Вызываем готовую безопасную функцию из модема
          send_ifkp_calibration_ladder(); 
          Serial.println(F("ОК"));
        }
        
        // IFKP: передача основного текста
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_IFKP] Текст: "));
          send_ifkp_string(my_text_variable + "\r\n"); 
        }
        
        // IFKP: передача первой телеметрии
        if (!pc_file_written && !soft_restart_flag) {
          String telemetry = get_telemetry_string();
          Serial.print(F("[ЭФИР_IFKP] Телем: "));
          send_ifkp_string(telemetry);
        }

        // ШАГ 4: Передача климатической телеметрии
        if (!pc_file_written && !soft_restart_flag) {
          if (device_BM[0] == 1) {
            String telemetry = get_climate_telemetry();
          Serial.print(F("[ЭФИР_IFKP] Телем: "));
            send_ifkp_string(telemetry);
          }
          else {
            Serial.println(F("[Система] отустствует климатический модуль. Погодная телеметрия не передаётся"));
          }
        }
        
        // IFKP завершение передачи
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_IFKP] Конец: "));
          send_ifkp_string("OVER.\r\n\r\n");
        }
        
        // Выводим информацию о длительности ТОЛЬКО при успешном и полном цикле сеанса
        if (!pc_file_written && !soft_restart_flag) {
          uint32_t ifkp_session_duration_sec = (millis() - ifkp_session_start_ms) / 1000;
          update_scheduler();
          char end_buf[128];
          snprintf(end_buf, sizeof(end_buf), "[Система] : %02d:%02d:%02d - Сеанс IFKP завершен. Длительность: %lu сек.", rtc_hour, rtc_min, rtc_sec, ifkp_session_duration_sec);
          Serial.println(end_buf);
          if (debug_flag) {
            // Пишем в файл на виртуальную флешку историю работы
            log_file_write_line("Сеанс IFKP завершен. Длительность: " + String(ifkp_session_duration_sec) + " сек.");
          }
        }
      } 
      
      // ГАРАНТИРОВАННЫЙ БЛОК ВЫХОДА ИЗ СЕАНСА
      is_transmitting = false;
      SI_POWER_OFF();
      dev_TX_state = false;
      update_scheduler(); // Гарантированно сдвигаем планировщик во избежание бесконечного перезапуска цикла
    } 
  }



  // =========================================================================
  // РЕЖИМ 1. СЕАНС СВЯЗИ RTTY
  // =========================================================================
  if (( (is_time_to_transmit(1) && !is_transmitting) || force_rtty_transmission) && !pc_file_written && !soft_restart_flag) {
    force_rtty_transmission = false;
    is_transmitting = true; 
    
    char time_buf[128];
    snprintf(time_buf, sizeof(time_buf), "[Система] %02d:%02d:%02d - Наступило время сеанса RTTY! Выходим в эфир...", rtc_hour, rtc_min, rtc_sec);
    Serial.println(time_buf);
    
    if ((my_call_variable.length() > 0 || my_text_variable.length() > 0) && !soft_restart_flag) {
      
      // Засекаем точное время старта сеанса
      uint32_t rtty_session_start_ms = millis();
      dev_TX_state = true;
      update_scheduler();
      SI_POWER_ON();
      
      if (!pc_file_written && !soft_restart_flag) {
        // --- НОВАЯ КОРРЕКТИРОВКА РАСЧЕТА ЧАСТОТ RTTY ---
        uint32_t rtty_mark_hz = strtoul(my_rtty_mark_var.c_str(), NULL, 10);
        if (rtty_mark_hz == 0)  rtty_mark_hz = 3601585; // Дефолтная частота MARK

        uint32_t rtty_shift_hz = strtoul(my_rtty_shift_var.c_str(), NULL, 10);
        if (rtty_shift_hz == 0) rtty_shift_hz = 170;    // Дефолтный сдвиг 170 Гц

        bool rtty_invert = (my_rtty_invert_var == "1"); // Флаг инверсии
        
        // Вызов обновленной функции модема
        prepare_rtty_frequencies(rtty_mark_hz, rtty_shift_hz, rtty_invert);
        
        Serial.print(F("[Скорость]: ")); Serial.print(1000000.0f / RTTY_BIT_TIME_US, 2); Serial.print(F(" БОД, "));
        Serial.print(F("длительность бита ")); Serial.print(RTTY_BIT_TIME_US / 1000); Serial.println(F(" мс"));
        
        // ШАГ 1: RTTY передача позывного и локатора
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_RTTY] Вызов: "));
          send_rtty_string("\r\n\r\nVVV BEACON DE " + my_call_variable + "/B DE " + my_call_variable + "/B, QTH " + my_qth_variable + " " + my_qth_variable + " \r\n");
        }
        
        // ШАГ 2: RTTY передача основного текста
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_RTTY] Текст: "));
          send_rtty_string(my_text_variable + " ");
        }
        
        // ШАГ 3: RTTY передача первой телеметрии
        if (!pc_file_written && !soft_restart_flag) {
          String telemetry = get_telemetry_string();
          Serial.print(F("[ЭФИР_RTTY] Телем: "));
          send_rtty_string(telemetry);
        }

        // ШАГ 4: Передача климатической телеметрии
        if (!pc_file_written && !soft_restart_flag) {
          if (device_BM[0] == 1) {
            String telemetry = get_climate_telemetry();
          Serial.print(F("[ЭФИР_RTTY] Телем: "));
            send_rtty_string(telemetry);
          }
          else {
            Serial.println(F("[Система] отустствует климатический модуль. Погодная телеметрия не передаётся"));
          }
        }
        
        // ШАГ 5: RTTY завершение передачи
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_RTTY] Конец: "));
          send_rtty_string("OVER.\r\n\r\n");
        }
        
        // Строка лога выводится СТРОГО при штатном завершении сеанса
        if (!pc_file_written && !soft_restart_flag) {
          uint32_t rtty_session_duration_sec = (millis() - rtty_session_start_ms) / 1000;
          update_scheduler();
          char end_buf[128];
          snprintf(end_buf, sizeof(end_buf), "[Система] : %02d:%02d:%02d - Сеанс RTTY завершен. Длительность: %lu сек.", rtc_hour, rtc_min, rtc_sec, rtty_session_duration_sec);
          Serial.println(end_buf);
        if (debug_flag) {
          // Пишем в файл на виртуальную флешку историю работы
          log_file_write_line("Сеанс RTTY завершен. Длительность: " + String(rtty_session_duration_sec) + " сек.");
          }
        }
      } 
      
      // ГАРАНТИРОВАННЫЙ БЛОК ВЫХОДА ИЗ СЕАНСА
      is_transmitting = false;
      SI_POWER_OFF();
      dev_TX_state = false;
      update_scheduler(); 
    } 
  }




  // =========================================================================
  // РЕЖИМ 2. СЕАНС СВЯЗИ CW
  // =========================================================================
  if (( (is_time_to_transmit(2) && !is_transmitting) || force_cw_transmission) && !pc_file_written && !soft_restart_flag) {
    force_cw_transmission = false;
    is_transmitting = true; 
    
    char time_buf[128];
    snprintf(time_buf, sizeof(time_buf), "[Система] %02d:%02d:%02d - Наступило время сеанса CW! Выходим в эфир...", rtc_hour, rtc_min, rtc_sec);
    Serial.println(time_buf);
    
    if ((my_call_variable.length() > 0 || my_text_variable.length() > 0) && !soft_restart_flag) {
      
      // Засекаем точное время старта сеанса в эфире
      uint32_t cw_session_start_ms = millis();
      dev_TX_state = true;
      update_scheduler();
      SI_POWER_ON();
      
      if (!pc_file_written && !soft_restart_flag) {
        uint32_t cw_hz = (my_freq_cw_var.length() > 0) ? strtoul(my_freq_cw_var.c_str(), NULL, 10) : 3601000;
        
        prepare_cw_frequency(cw_hz);
        Serial.print(F("[Скорость] ")); Serial.print(1200 / CW_DOT_TIME_MS); Serial.print(F(" WPM, "));
        Serial.print(F("длительность точки ")); Serial.print(CW_DOT_TIME_MS); Serial.println(F(" мс"));
        
        // ШАГ 1: Передача позывного и локатора
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_CW] Вызов: "));
          send_cw_string("VVV BEACON DE " + my_call_variable + "/B DE " + my_call_variable + "/B, QTH " + my_qth_variable + " " + my_qth_variable + " ");
        }
        
        // ШАГ 2: Передача основного текста
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_CW] Текст: "));
          send_cw_string(my_text_variable + " ");
        }
        
        // ШАГ 3: Передача первой телеметрии
        if (!pc_file_written && !soft_restart_flag) {
          String telemetry = get_telemetry_string();
          Serial.print(F("[ЭФИР_CW] Телем: "));
          send_cw_string(telemetry);
        }
        
        // ШАГ 4: Передача климатической телеметрии
        if (!pc_file_written && !soft_restart_flag) {
          if (device_BM[0] == 1) {
            String telemetry = get_climate_telemetry();
            Serial.print(F("[ЭФИР_CW] Телем: "));
            send_cw_string(telemetry);
          }
          else {
            Serial.println(F("[Система] отустствует климатический модуль. Погодная телеметрия не передаётся"));
          }
        }
        
        // ШАГ 5: Завершение передачи
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_CW] Конец: "));
          send_cw_string("OVER.");
        }
        
        // Считаем, сколько полных секунд длился сеанс
        uint32_t cw_session_duration_sec = (millis() - cw_session_start_ms) / 1000;
        update_scheduler();
        char end_buf[128];
        snprintf(end_buf, sizeof(end_buf), "[Система] %02d:%02d:%02d - Сеанс CW завершен. Длительность: %lu сек.", rtc_hour, rtc_min, rtc_sec, cw_session_duration_sec);
        Serial.println(end_buf);

        if (debug_flag) {
          // Пишем в файл на виртуальную флешку историю работы
          log_file_write_line("Сеанс CW завершен.   Длительность: " + String(cw_session_duration_sec) + " сек.");
        }
      } 
      
      // БЛОК ВЫХОДА ИЗ СЕАНСА - выполняется всегда: и при успехе, и при экстренном прерывании
      is_transmitting = false; 
      SI_POWER_OFF();
      dev_TX_state = false;
      update_scheduler(); // Переключаем планировщик на следующий интервал времени
    } 
  }

  // =========================================================================
  // РЕЖИМ 3. ЦЕПОЧКА
  // =========================================================================
  // Проверка запуска трехмодовой цепочки из расписания
  if (is_time_to_transmit(3) && !is_transmitting) {
       Serial.println(F("[Планировщик] Время подошло. Запуск сквозной цепочки CW -> RTTY -> IFKP"));
       is_transmitting = true;
       usr_chain_state = USR_START_CW; // Толкаем ваш штатный автомат, он всё сделает сам!
  }


  delay(1); 
}


// RP2040_beacon.ino — выравнивание колонок в отчёте [SCAN_RESULT]  
static String fmt_dev(const char* name, const char* addr, int sda, int scl) {  
  char buf[48];  
  // %-8s — имя слева до 8 симв.; %-4s — адрес; %2d — пины по правому краю  
  snprintf(buf, sizeof(buf), "%-8s(%-4s SDA:%2d SCL:%2d)", name, addr, sda, scl);  
  return String(buf);  
}

// Сбор результатов сканирования с фиксацией физических пинов SDA/SCL и их экспорт в файл конфигурации железа
void I2C_Scanner() {
  // Первично производим аппаратный обход портов I2C
  scanRP2040Ports();
  printDeviceTable();

  // Формируем строку отчёта. Каждое устройство пишется с новой строки (\r\n)
  String results = "";
  
  if (device_SI[0] == 1) {
    if (results.length() > 0) results += "\r\n";
    results += fmt_dev("SI5351", "0x60", device_SI[2], device_SI[3]);
  }
  if (device_DS[0] == 1) {
    if (results.length() > 0) results += "\r\n";
    results += fmt_dev(rtc_chip_name.c_str(), "0x68", device_DS[2], device_DS[3]); 
  }
  if (device_AT[0] == 1) {
    if (results.length() > 0) results += "\r\n";
    char addr_buf[8];
    snprintf(addr_buf, sizeof(addr_buf), "0x%02X", device_AT[4]);
    results += fmt_dev("EEPROM", addr_buf, device_AT[2], device_AT[3]);
  }
  if (device_BM[0] == 1) {
    if (results.length() > 0) results += "\r\n";
    char addr_buf[8];
    snprintf(addr_buf, sizeof(addr_buf), "0x%02X", device_BM[4]);
    results += fmt_dev("BME/BMP", addr_buf, device_BM[2], device_BM[3]);
  }
  if (device_DL[0] == 1) {
    if (results.length() > 0) results += "\r\n";
    results += fmt_dev("LCD", "0x27", device_DL[2], device_DL[3]);
  }
  
  results.trim();
  if (results.length() == 0) {
    results = "Устройства на шине I2C не обнаружены";
  }

  // Записываем результат и обновляем файл на диске
  save_hardware_settings_to_file(results);
  Serial.println(F("[Система] Результаты I2C сканирования экспортированы в SET.TXT"));
}

void scanRP2040Ports() {
  // Serial.println(F("\n=== ЗАПУСК ПОЛНОГО СКАНИРОВАНИЯ РАЗРЕШЕННЫХ ПОРТОВ RP2040-ZERO ==="));

  // Шаг 1: Сканируем шину i2c0 
  // SDA: 0, SCL: 1 (Удобные боковые пины)
  // SDA: 4, SCL: 5 (Удобные боковые пины)
  // SDA: 8, SCL: 9 (Удобные боковые пины)
  // SDA: 12, SCL: 13 (Удобные боковые пины)
  // SDA: 20, SCL: 21 (Задние площадки под пайку)

  for (uint8_t sda = 0; sda <= 27; sda += 4) {
    uint8_t scl = sda + 1;
    
    // ИСКЛЮЧЕНИЯ ДЛЯ RP2040-ZERO:
    //if (sda == 16 || scl == 16) continue;
    //if (sda >= 23 && sda <= 25) continue;
    //if (scl >= 23 && scl <= 25) continue;

    // Проверяем кастомный список исключений из SET.TXT
    if (is_pin_excluded_from_scan(sda) || is_pin_excluded_from_scan(scl)) {
      //Serial.print(F("[Сканер]  Пропуск исключенных пинов: ")); Serial.print(sda); Serial.print(F(", ")); Serial.println(scl);
      continue;
    }
    I2C_Scan_module(0, sda, scl, false); 
    delay(50);
  }

  // Шаг 2: Сканируем шину i2c1
  // SDA: 2, SCL: 3 (Удобные боковые пины)
  // SDA: 6, SCL: 7 (Удобные боковые пины)
  // SDA: 10, SCL: 11 (Удобные боковые пины)
  // SDA: 14, SCL: 15 (Удобные боковые пины)
  // SDA: 18, SCL: 19 (Задние площадки под пайку)
  // SDA: 26, SCL: 27 (Нижние аналоговые пины)
  for (uint8_t sda = 2; sda <= 26; sda += 4) {
    uint8_t scl = sda + 1;
    if (is_pin_excluded_from_scan(sda) || is_pin_excluded_from_scan(scl)) {
      //Serial.print(F("[Сканер]  Пропуск исключенных пинов: ")); Serial.print(sda); Serial.print(F(", ")); Serial.println(scl);
      continue;
    }
    I2C_Scan_module(1, sda, scl, false); 
    delay(50);
  }
}

void I2C_Scan_module(int WIRE_NO, int PIN_SDA, int PIN_SCL, bool LOGGING) {
  watchdog_update(); // сброс сторожевого таймера
  // сканер устройств на указанной шине I2C
  byte error, address;
  int nDevices = 0;

  // Выбираем нужный интерфейс Wire в зависимости от WIRE_NO
  TwoWire *pWire = (WIRE_NO == 1) ? &Wire1 : &Wire;

  // Настройка пинов и старт шины I2C
  pWire->end();
  pWire->setSDA(PIN_SDA);
  pWire->setSCL(PIN_SCL);
  pWire->begin();
  pWire->setClock(400000);

  if (LOGGING) {
    Serial.print("[Система] Сканирование шины I2C-"); Serial.print(WIRE_NO); Serial.print(" на пинах SDA=");
    Serial.print(PIN_SDA); Serial.print(", SCL="); Serial.print(PIN_SCL); Serial.println(":");
  }

  for (address = 1; address < 127; address++) {
    // Первичная проверка присутствия прибора по ACK
    pWire->beginTransmission(address);
    error = pWire->endTransmission();

    if (error == 0) {
      // Инициализируем локальный флаг глубокой проверки
      bool hardware_verified = false;

      // =========================================================================
      // 1. УСИЛЕННАЯ ВЕРИФИКАЦИЯ СИНТЕЗАТОРА SI5351 (0x60)
      // =========================================================================
      if (address == 0x60) {
        byte revid = 0;
        pWire->beginTransmission(address);
        pWire->write(0x00); // Регистр ревизии чипа / Device Status
        if (pWire->endTransmission() == 0) {
          // Запрашиваем 1 байт и строго проверяем успешность транзакции шины
          if (pWire->requestFrom(address, (uint8_t)1) == 1 && pWire->available()) {
            byte reg0 = pWire->read();
            revid = reg0 & 0x03;
            hardware_verified = true; // Устройство физически ответило байтом данных!
            
            if (LOGGING) { 
              Serial.print("[Система] - найден прибор 0x60: Si5351 ревизии "); 
              Serial.println(revid); 
            }
          }
        }
        
        if (hardware_verified) {
          device_SI[0] = 1;
          device_SI[1] = WIRE_NO;
          device_SI[2] = PIN_SDA;
          device_SI[3] = PIN_SCL;
          device_SI[4] = address;
          device_SI_name = "SI5351 Генератор";
        }
      }

      // =========================================================================
      // 2. УСИЛЕННАЯ ВЕРИФИКАЦИЯ ЧАСОВ RTC DS3231 / DS1307 (0x68)
      // =========================================================================
      else if (address == 0x68) {
        extern String rtc_chip_name;
        pWire->beginTransmission(address);
        pWire->write(0x0F); // Регистр управления/статуса
        pWire->write(0x70); 
        if (pWire->endTransmission() == 0) {
          pWire->beginTransmission(address);
          pWire->write(0x0F);
          pWire->endTransmission();
          
          // Проверяем, вернулся ли ровно 1 байт без обрыва связи
          if (pWire->requestFrom(address, (uint8_t)1) == 1 && pWire->available()) {
            uint8_t testByte = pWire->read();
            hardware_verified = true; // Пробное чтение прошло успешно

            if ((testByte & 0x70) == 0x70) {
              rtc_chip_name = "DS1307";
              pWire->beginTransmission(address);
              pWire->write(0x0F); pWire->write(0x00); // Чистим NVRAM регистр за собой
              pWire->endTransmission();
            } else {
              rtc_chip_name = "DS3231";
            }
            if (LOGGING) { 
              Serial.print("[Система] - найден прибор 0x68: Часы "); 
              Serial.println(rtc_chip_name); 
            }
          }
        }
        
        if (hardware_verified) {
          device_DS[0] = 1;
          device_DS[1] = WIRE_NO;
          device_DS[2] = PIN_SDA;
          device_DS[3] = PIN_SCL;
          device_DS[4] = address;
          device_DS_name = rtc_chip_name + " Часы RTC";
        }
      }

      // =========================================================================
      // 3. УСИЛЕННАЯ ВЕРИФИКАЦИЯ ЭНЕРГОНЕЗАВИСИМОЙ ПАМЯТИ AT24Cxx (0x50..0x57)
      // =========================================================================
      else if (address >= 0x50 && address <= 0x57) {
        pWire->beginTransmission(address);
        pWire->write(0x00); pWire->write(0x00); // Выставляем указатель на адрес ячейки 0
        if (pWire->endTransmission() == 0) {
          // Выполняем строгое верификационное чтение первого байта памяти
          if (pWire->requestFrom(address, (uint8_t)1) == 1 && pWire->available()) {
            uint8_t byte0 = pWire->read();
            hardware_verified = true; // Микросхема памяти подтверждена!

            int kbits = 512; 
            int sizesToTest[] = {32, 64, 128, 256}; 

            for (int i = 0; i < 4; i++) {
              uint16_t testAddr = (sizesToTest[i] * 128); 
              pWire->beginTransmission(address);
              pWire->write((uint8_t)(testAddr >> 8)); pWire->write((uint8_t)(testAddr & 0xFF));
              
              if (pWire->endTransmission() == 0 && pWire->requestFrom(address, (uint8_t)1) == 1 && pWire->available()) {
                uint8_t testByte = pWire->read();
                if (testByte == byte0) {
                  pWire->beginTransmission(address);
                  pWire->write(0x00); pWire->write(0x00); pWire->write((uint8_t)~byte0);
                  pWire->endTransmission();
                  delay(5); 

                  pWire->beginTransmission(address);
                  pWire->write((uint8_t)(testAddr >> 8)); pWire->write((uint8_t)(testAddr & 0xFF));
                  pWire->endTransmission();
                  
                  if (pWire->requestFrom(address, (uint8_t)1) == 1 && pWire->available() && pWire->read() == (uint8_t)~byte0) {
                    kbits = sizesToTest[i];
                    pWire->beginTransmission(address);
                    pWire->write(0x00); pWire->write(0x00); pWire->write(byte0);
                    pWire->endTransmission();
                    delay(5);
                    break;
                  }
                  pWire->beginTransmission(address);
                  pWire->write(0x00); pWire->write(0x00); pWire->write(byte0);
                  pWire->endTransmission();
                  delay(5);
                }
              }
            }
            if (LOGGING) {
              Serial.print("[Система] - найден прибор 0x"); Serial.print(address, HEX);
              Serial.print(": EEPROM AT24C"); Serial.print(kbits);
              Serial.print(" ("); Serial.print(kbits / 8); Serial.println(" Кбайт)");
            }
            device_AT[0] = 1;
            device_AT[1] = WIRE_NO;
            device_AT[2] = PIN_SDA;
            device_AT[3] = PIN_SCL;
            device_AT[4] = address;
            device_AT_name = "EEPROM AT24C" + String(kbits) + " " + String(kbits/8) + "Кб";
          }
        }
      }

      // =========================================================================
      // 4. УСИЛЕННАЯ ВЕРИФИКАЦИЯ ДИСПЛЕЯ LCD1602 (0x27) ЧЕРЕЗ ПРОВЕРКУ ФЛАНГА ШИНЫ
      // =========================================================================
      else if (address == 0x27) {
        // Микросхема расширителя PCF8574 всегда возвращает 1 байт состояния пинов при чтении
        if (pWire->requestFrom(address, (uint8_t)1) == 1) {
          hardware_verified = true; // Экран подтвердил своё присутствие

          if (LOGGING) { Serial.println("[Система] - найден прибор 0x27: LCD Дисплей (PCF8574). Инициализация..."); }
          device_DL[0] = 1;
          device_DL[1] = WIRE_NO;
          device_DL[2] = PIN_SDA;
          device_DL[3] = PIN_SCL;
          device_DL[4] = address;
          device_DL_name = "H44780 LCD Дисплей";
          
          pWire->end();
          LCD_init(true);
          LCD_print("I2C Scan: OK!", 0, 0);
          LCD_print("Display Active", 1, 0);
          pWire->begin();
          pWire->setClock(400000);
        }
      }

      // =========================================================================
      // 5. УСИЛЕННАЯ ВЕРИФИКАЦИЯ КЛИМАТИЧЕСКИХ ДАТЧИКОВ BME280 / BMP280 / BMP180
      // =========================================================================
      else if (address == 0x76 || address == 0x77) {
        uint8_t reg_id_addr = (address == 0x76 || address == 0x77) ? 0xD0 : 0xD0; // Идентификационный регистр Chip ID
        pWire->beginTransmission(address);
        pWire->write(reg_id_addr);
        if (pWire->endTransmission() == 0) {
          // Запрашиваем уникальный заводской ID датчика
          if (pWire->requestFrom(address, (uint8_t)1) == 1 && pWire->available()) {
            uint8_t chipID = pWire->read();
            hardware_verified = true; // Датчик физически прочитан!

            if (chipID == 0x60) {
              device_BM_name = "BME280 Гигрометр";
            } else if (chipID == 0x58 || chipID == 0x56 || chipID == 0x57) {
              device_BM_name = "BMP280 Барометр";
            } else if (chipID == 0x55 && address == 0x77) {
              device_BM_name = "BMP180 Барометр";
            } else {
              device_BM_name = "Неизвестный климатический чип ID:0x" + String(chipID, HEX);
            }

            if (LOGGING) { 
              Serial.print("[Система] - найден прибор 0x"); Serial.print(address, HEX);
              Serial.print(": "); Serial.println(device_BM_name); 
            }

            device_BM[0] = 1;
            device_BM[1] = WIRE_NO;
            device_BM[2] = PIN_SDA;
            device_BM[3] = PIN_SCL;
            device_BM[4] = address;
          }
        }
      }

      // Финальный инкремент счетчика устройств только при успешном прохождении глубокого теста
      if (hardware_verified) {
        nDevices++;
      } else {
        if (LOGGING) {
          Serial.print("[Предупреждение] Фантомный ACK на адресе 0x");
          Serial.print(address, HEX);
          Serial.println("! Устройство проигнорировано (нет ответа данных).");
        }
      }
    }
    else if (error == 4) {
      Serial.print("[Система] Критическая ошибка шины на адресе: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    if (LOGGING) {Serial.print("[Система] - I2C устройства не найдены.\n");}
  }

  // останов шины I2C
  pWire->end();
}

void printDeviceTable() {
  char buf[96]; // Буфер для форматирования строки

  // Чистый заголовок без вертикальных рамок
  Serial.println(F("=========================================================================="));
  Serial.println(F("  Устройство   PIN SDA   PIN SCL   Wire No   Address     Имя"));
  Serial.println(F("--------------------------------------------------------------------------"));

  uint8_t* arrays[] = {device_SI, device_DS, device_AT, device_BM, device_DL};
  const char* names[] = {"device_SI", "device_DS", "device_AT", "device_BM", "device_DL"};
  String* deviceNames[] = {&device_SI_name, &device_DS_name, &device_AT_name, &device_BM_name, &device_DL_name};

  for (int i = 0; i < 5; i++) {
    // Если первый элемент (Active) равен 0, пропускаем вывод всей строки
    if (arrays[i][0] == 0) {
      continue; 
    }

    // Выводим данные с фиксированными отступами, а имя просто дописываем в конце
    snprintf(buf, sizeof(buf), 
             "  %-10s    %2d        %2d        %d        0x%02X       ", 
             names[i], 
             arrays[i][2], // PIN_SDA
             arrays[i][3], // PIN_SCL
             arrays[i][1], // WIRE_NO (номер шины)
             arrays[i][4]);// address
             
    Serial.print(buf);
    Serial.println(*deviceNames[i]); // Выводим String-имя в самый правый край
  }
  Serial.println(F("=========================================================================="));
  
      LCD_init(true);      
}

