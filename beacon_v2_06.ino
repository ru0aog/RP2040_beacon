#include "file_manager.h"
#include "si5351_driver.h"
#include "ifkp_modem.h"
#include "rtty_modem.h"  
#include "scheduler.h"
#include "cw_modem.h"
#include "bme280.h"
#include "gen_fract.h"
#include "LCD.h"
#include <hardware/watchdog.h>
#include <hardware/adc.h>
#include <Adafruit_TinyUSB.h>

// автоматический маяк на RP2040
// версия 2.06 от 2026-09-11, автор RU0AOG
// моды CW, RTTY, IFKP
// сканирование подключенного оборудования
// датчик давления BME/BMP280
// дисковая система с файлом конфигурации
// редактирование конфигурации из консоли
// работа по расписанию
// передача данных телеметрии

#define BCN_VER 2.06
#define BCN_DAT "2026-09-11"

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

const uint8_t BME_POWER_PIN = 11;
const uint8_t SI_POWER_PIN = 17;
const uint8_t DS_POWER_PIN = 16;

bool force_cw_transmission   = false; // Флаг ручного запуска CW
bool force_rtty_transmission = false; // Флаг ручного запуска RTTY
bool force_ifkp_transmission = false; // Флаг ручного запуска IFKP

bool soft_restart_flag = false;
extern bool pc_activity_detected; 
bool is_transmitting = false; // Флаг передачи 
String rtc_chip_name = "Неизвестный RTC"; // Сюда сканер запишет точное имя чипа

extern uint8_t rtc_sec;

// Функция проверки и обработки текстовых команд с локальным эхом
extern void update_info_config_from_console(String marker, String new_value);

void check_serial_commands() {
  static char cmd_buffer[64];
  static size_t buf_idx = 0;

  // Цикл работает, пока не вычитает ВСЕ доступные символы из UART
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (buf_idx > 0) {
        cmd_buffer[buf_idx] = '\0'; 
        String command = String(cmd_buffer); 
        // Сразу сбрасываем буфер, так как команда скопирована в String
        buf_idx = 0; 

        command.trim(); 

        if (command.length() > 0) {
          Serial.print(F("\r\n> ")); 
          Serial.println(command);
          
          LCD_print("                    ", 1, 0);
          LCD_print(">"+command, 1, 0);

          if (command.equalsIgnoreCase("?")) {
            print_current_settings();
            print_current_date();
            print_current_time();
            Serial.print(F("Телеметрия: ")); Serial.println(get_telemetry_string());
            Serial.print(F("Телеметрия: ")); Serial.println(get_climate_telemetry());
            Serial.println(F("Введите help для перехода в справочное меню по командам управления\r\n"));
          }
          else if (command.equalsIgnoreCase("TELE")) {
            print_current_time();
            Serial.print(F("Телеметрия: ")); Serial.println(get_telemetry_string());
            Serial.print(F("Телеметрия: ")); Serial.println(get_climate_telemetry());
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
            Serial.flush();
            SI_POWER_OFF();
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
          Serial.println(F("start cw        - Немедленно запустить внеочередной сеанс CW"));
          Serial.println(F("start rtty      - Немедленно запустить внеочередной сеанс RTTY"));
          Serial.println(F("start ifkp      - Немедленно запустить внеочередной сеанс IFKP"));
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
      }
    }
  } // Конец while
}

void setup() {
  adc_init();
  init_file_manager();  // инициализировать флэш-диск

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
  Serial.println("[Система] Последовательное соединение восстановлено.");
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("[Система] "); Serial.println(my_FAT);

  //Serial.print("[Система] Сканирование доступных устройств. \n");
  I2C_Scanner();
  printDeviceTable();
  //Serial.print("[Система] Сканирование завершено.\n");

  init_BME();          // инициализировать bme280
  //BME_read();

  init_scheduler();    // инициализировать дс3231

  init_si5351();  // инициализировать си5351

  //Serial.println(F("\n================================================================"));
  Serial.println(F("  АВТОМАТИЧЕСКИЙ РАДИОМАЯК ЗАПУЩЕН"));
  Serial.print(F("  версия "));
  Serial.print(BCN_VER);
  Serial.print(F(" прошивка от "));
  Serial.println(BCN_DAT);
  Serial.println(F("  Плата готова к работе."));
  Serial.print(F("  Текущая дата ")); Serial.print(get_current_date()); Serial.print(F(" время ")); Serial.println(get_current_time());
  Serial.println(F("  Введите команду или h для выхода в справочное меню."));
  Serial.println(F("==========================================================================\n"));

}

void loop() {
  #if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
  TinyUSBDevice.task(); 
  #endif

  extern uint8_t rtc_sec; 
  // =========================================================================
  // 1. ОБРАБОТКА МЯГКОГО РЕСТАРТА
  // =========================================================================
  if (soft_restart_flag) {
    soft_restart_flag = false; 
    current_tone = 0; 
    delay(100);

    Serial.println("\r\nПоследовательное соединение: OK");

    read_file_to_variable(); // Извлекаем чистые маркеры
    pc_file_written = false;
    print_current_settings();
    return; // Сразу уходим на новый виток loop, сбрасывая старые флаги
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

      LCD_init(false);      
      String T_DS_text  = "T1=" + get_telemetry_string().substring(5, 9);
      String T_CPU_text = "CPU " + get_telemetry_string().substring(17, 21) + " ";
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
  if ((is_time_to_transmit(0) || force_ifkp_transmission) && !pc_file_written && !soft_restart_flag) {
    force_ifkp_transmission = false;
    is_transmitting = true;
    
    char time_buf[128];
    snprintf(time_buf, sizeof(time_buf), "\r\n[Система] : %02d:%02d:%02d - Наступило время сеанса IFKP! Выходим в эфир...", rtc_hour, rtc_min, rtc_sec);
    Serial.println(time_buf);
    
    if ((my_call_variable.length() > 0 || my_text_variable.length() > 0) && !soft_restart_flag) {
      
      // Засекаем точное время старта сеанса связи IFKP
      uint32_t ifkp_session_start_ms = millis();
      SI_POWER_ON();
      
      if (SI_FAIL == false && !pc_file_written && !soft_restart_flag) {
        uint32_t ifkp_hz = strtoul(my_freq_ifkp_var.c_str(), NULL, 10);
        if (ifkp_hz == 0) ifkp_hz = 3601307; 
        
        // Вывод параметров IFKP
        prepare_ifkp_frequencies(ifkp_hz);
        
        // IFKP: передача стартовой лесенки
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_IFKP] Синхр: "));
          for(int i = 0; i < 10; i++) {
              if (pc_file_written || soft_restart_flag) break; 
              send_delta(1); 
              Serial.print(F(".")); 
          }
          Serial.println(F("ОК"));
        }
        
        // IFKP: передача позывного и локатора
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_IFKP] Вызов: "));
          send_ifkp_string("\r\n\r\nVVV BEACON DE " + my_call_variable + "/B DE " + my_call_variable + "/B, QTH " + my_qth_variable + " " + my_qth_variable + " \r\n"); 
        }
        
        // IFKP: передача лесенки
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_IFKP] Синхр: "));
          for(int i = 0; i < 10; i++) {
              if (pc_file_written || soft_restart_flag) break; 
              send_delta(1); 
              Serial.print(F(".")); 
          }
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
        
        // IFKP: передача климатической телеметрии
        if (!pc_file_written && !soft_restart_flag) {
          String telemetry = get_climate_telemetry();
          Serial.print(F("[ЭФИР_IFKP] Телем: "));
          send_ifkp_string(telemetry);
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
        }
      } 
      
      // ГАРАНТИРОВАННЫЙ БЛОК ВЫХОДА ИЗ СЕАНСА
      is_transmitting = false;
      SI_POWER_OFF();
      update_scheduler(); // Гарантированно сдвигаем планировщик во избежание бесконечного перезапуска цикла
      Serial.println("");
    } 
  }



  // =========================================================================
  // РЕЖИМ 1. СЕАНС СВЯЗИ RTTY
  // =========================================================================
  if ((is_time_to_transmit(1) || force_rtty_transmission) && !pc_file_written && !soft_restart_flag) {
    force_rtty_transmission = false;
    is_transmitting = true; 
    
    char time_buf[128];
    snprintf(time_buf, sizeof(time_buf), "\r\n[Система] : %02d:%02d:%02d - Наступило время сеанса RTTY! Выходим в эфир...", rtc_hour, rtc_min, rtc_sec);
    Serial.println(time_buf);
    
    if ((my_call_variable.length() > 0 || my_text_variable.length() > 0) && !soft_restart_flag) {
      
      // Засекаем точное время старта сеанса
      uint32_t rtty_session_start_ms = millis();
      SI_POWER_ON();
      
      if (SI_FAIL == false && !pc_file_written && !soft_restart_flag) {
        uint32_t rtty_space_hz = strtoul(my_rtty_space_var.c_str(), NULL, 10);
        if (rtty_space_hz == 0) rtty_space_hz = 3601000; 
        uint32_t rtty_mark_hz  = strtoul(my_rtty_mark_var.c_str(), NULL, 10);
        if (rtty_mark_hz == 0)  rtty_mark_hz = 3601170;  
        
        prepare_rtty_frequencies(rtty_space_hz, rtty_mark_hz);
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
        
        // ШАГ 4: RTTY передача климатической телеметрии
        if (!pc_file_written && !soft_restart_flag) {
          String telemetry = get_climate_telemetry();
          Serial.print(F("[ЭФИР_RTTY] Телем: "));
          send_rtty_string(telemetry);
        }
        
        // ШАГ 5: RTTY завершение передачи
        if (!pc_file_written && !soft_restart_flag) {
          Serial.print(F("[ЭФИР_RTTY] Конец: "));
          send_rtty_string("OVER.\r\n\r\n");
        }
        
        // Строка лога выводится СТРОГО при штатном завершении сеанса (нет флага прерывания)
        if (!pc_file_written && !soft_restart_flag) {
          uint32_t rtty_session_duration_sec = (millis() - rtty_session_start_ms) / 1000;
          update_scheduler();
          char end_buf[128];
          snprintf(end_buf, sizeof(end_buf), "[Система] : %02d:%02d:%02d - Сеанс RTTY завершен. Длительность: %lu сек.", rtc_hour, rtc_min, rtc_sec, rtty_session_duration_sec);
          Serial.println(end_buf);
        }
      } 
      
      // ГАРАНТИРОВАННЫЙ БЛОК ВЫХОДА ИЗ СЕАНСА
      is_transmitting = false;
      SI_POWER_OFF();
      update_scheduler(); // Гарантированно сдвигаем планировщик, чтобы избежать зацикливания
      Serial.println("");
    } 
  }



  // =========================================================================
  // РЕЖИМ 2. СЕАНС СВЯЗИ CW
  // =========================================================================
  if ((is_time_to_transmit(2) || force_cw_transmission) && !pc_file_written && !soft_restart_flag) {
    force_cw_transmission = false;
    is_transmitting = true; 
    
    char time_buf[128];
    snprintf(time_buf, sizeof(time_buf), "\r\n[Система] : %02d:%02d:%02d - Наступило время сеанса CW! Выходим в эфир...", rtc_hour, rtc_min, rtc_sec);
    Serial.println(time_buf);
    
    if ((my_call_variable.length() > 0 || my_text_variable.length() > 0) && !soft_restart_flag) {
      
      // Засекаем точное время старта сеанса в эфире
      uint32_t cw_session_start_ms = millis();

      SI_POWER_ON();
      
      if (SI_FAIL == false && !pc_file_written && !soft_restart_flag) {
        uint32_t cw_hz = (my_freq_cw_var.length() > 0) ? strtoul(my_freq_cw_var.c_str(), NULL, 10) : 3601000;
        
        prepare_cw_frequency(cw_hz);
        Serial.print(F("[Скорость]: ")); Serial.print(1200 / CW_DOT_TIME_MS); Serial.print(F(" WPM, "));
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
          String telemetry = get_climate_telemetry();
          Serial.print(F("[ЭФИР_CW] Телем: "));
          send_cw_string(telemetry);
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
        snprintf(end_buf, sizeof(end_buf), "[Система] : %02d:%02d:%02d - Сеанс CW завершен. Длительность: %lu сек.", rtc_hour, rtc_min, rtc_sec, cw_session_duration_sec);
        Serial.println(end_buf);
      } 
      
      // БЛОК ВЫХОДА ИЗ СЕАНСА - выполняется всегда: и при успехе, и при экстренном прерывании
      is_transmitting = false; 
      SI_POWER_OFF(); 
      update_scheduler(); // Переключаем планировщик на следующий интервал времени
      Serial.println("");
    } 
  }


  delay(1); 
}

void I2C_Scanner() {
  pinMode(DS_POWER_PIN, OUTPUT);
  digitalWrite(DS_POWER_PIN, HIGH);
  pinMode(BME_POWER_PIN, OUTPUT);
  digitalWrite(BME_POWER_PIN, HIGH);
  delay(100); // Даем чипам время на аппаратный старт
  
  scanRP2040Ports();

}

void scanRP2040Ports() {
  //Serial.println(F("\n=== ЗАПУСК ПОЛНОГО СКАНИРОВАНИЯ РАЗРЕШЕННЫХ ПОРТОВ RP2040 ==="));

  // Шаг 1: Сканируем шину i2c0 (SDA: 0, 4, 8, 12, 16, 20, 24, 28)
  //Serial.println(F("\nПроверка аппаратной шины I2C_0:"));
  for (uint8_t sda = 0; sda <= 28; sda += 4) {
    uint8_t scl = sda + 1;
    
    // ИСКЛЮЧЕНИЯ:
    // 1. Исключаем пины выше 22 (т.е. 23 и больше)
    if (sda > 22 || scl > 22) continue;
    // 2. Исключаем конкретные пины: 11, 16, 17
    if (sda == 11 || sda == 16 || sda == 17) continue;
    if (scl == 11 || scl == 16 || scl == 17) continue;

    //Serial.print(F("Сканирую шину 0 на пинах SDA:"));
    //Serial.print(sda);
    //Serial.print(F(", SCL:"));
    //Serial.println(scl);

    I2C_Scan_module(0, sda, scl, false); 
    delay(50);
  }

  // Шаг 2: Сканируем шину i2c1 (SDA: 2, 6, 10, 14, 18, 22, 26)
  //Serial.println(F("\nПроверка аппаратной шины I2C_1:"));
  for (uint8_t sda = 2; sda <= 26; sda += 4) {
    uint8_t scl = sda + 1;

    // ИСКЛЮЧЕНИЯ:
    // 1. Исключаем пины выше 22
    if (sda > 22 || scl > 22) continue;
    // 2. Исключаем конкретные пины: 11, 16, 17
    if (sda == 11 || sda == 16 || sda == 17) continue;
    if (scl == 11 || scl == 16 || scl == 17) continue;

    //Serial.print(F("Сканирую шину 1 на пинах SDA:"));
    //Serial.print(sda);
    //Serial.print(F(", SCL:"));
    //Serial.println(scl);

    I2C_Scan_module(1, sda, scl, false); 
    delay(50);
  }

  //Serial.println(F("\n=== СКАНИРОВАНИЕ ЗАВЕРШЕНО ==="));
}

void I2C_Scan_module(int WIRE_NO, int PIN_SDA, int PIN_SCL, bool LOGGING) {
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
    // Начало передачи по адресу
    pWire->beginTransmission(address);
    error = pWire->endTransmission();

    if (error == 0) {
      if (LOGGING) {
        Serial.print("[Система] - найден I2C прибор по адресу: 0x");
        if (address < 16) Serial.print("0");
        Serial.print(address, HEX);
      }
      
      // для Si5351
      if (address == 0x60) {
      byte revid;
        pWire->beginTransmission(address);
        pWire->write(0x00);
        if (pWire->endTransmission() == 0) {
          pWire->requestFrom(address, 1);
          if (pWire->available()) {
            byte reg0 = pWire->read();
            revid = reg0 & 0x03;
            if (LOGGING) {Serial.print(" - это чип Si5351 ревизии "); Serial.print(revid);}
          }
        }
        device_SI[0] = 1;
        device_SI[1] = WIRE_NO;
        device_SI[2] = PIN_SDA;
        device_SI[3] = PIN_SCL;
        device_SI[4] = address;
        device_SI_name = "SI5351 Генератор";
      }

      // для DS3231/1307
      if (address == 0x68) {
        extern String rtc_chip_name;
        pWire->beginTransmission(address);
        pWire->write(0x11); // запрос регистра температуры ds3231
        if (pWire->endTransmission() == 0) {
          pWire->requestFrom(address, 1);
          if (pWire->available()) {
            byte tempMSB = pWire->read();
            if (tempMSB < 85 || tempMSB > 215) {
              if (LOGGING) {Serial.print(" - это чип DS3231, температура "); Serial.print(tempMSB); Serial.print(" C");}
            rtc_chip_name = "DS3231";
            } else {
                if (LOGGING) {Serial.print(" - это чип DS1307");}
              rtc_chip_name = "DS1307";
            }
          }
        }
        device_DS[0] = 1;
        device_DS[1] = WIRE_NO;
        device_DS[2] = PIN_SDA;
        device_DS[3] = PIN_SCL;
        device_DS[4] = address;
        device_DS_name = rtc_chip_name + " Часы RTC";
      }

      // для флэш памяти AT24Cxx
      // Проверяем весь диапазон адресов для памяти AT24Cxx (от 0x50 до 0x57)
      if (address >= 0x50 && address <= 0x57) {
        
        // ТЕСТОВОЕ ЧТЕНИЕ: проверяем, отвечает ли чип вообще
        pWire->beginTransmission(address);
        pWire->write(0x00); // Старший байт адреса 0
        pWire->write(0x00); // Младший байт адреса 0
        
        if (pWire->endTransmission() == 0) {
          pWire->requestFrom(address, 1);
          if (pWire->available()) {
            uint8_t byte0 = pWire->read(); // Запоминаем значение в ячейке 0

            // Переменная для хранения определенного объема памяти в Килобитах
            int kbits = 0; 
            
            // Массив возможных объемов для проверки переполнения: 32, 64, 128, 256, 512 Кбит
            int sizesToTest[] = {32, 64, 128, 256}; 
            
            // По умолчанию предполагаем самый большой чип, если переполнение не подтвердится
            kbits = 512; 

            for (int i = 0; i < 4; i++) {
              // Вычисляем адрес ячейки, где должно случиться зеркалирование (объем в Кбитах * 1024 / 8 битов)
              uint16_t testAddr = (sizesToTest[i] * 128); 

              pWire->beginTransmission(address);
              pWire->write((uint8_t)(testAddr >> 8));   // Старший байт тестового адреса
              pWire->write((uint8_t)(testAddr & 0xFF));  // Младший байт тестового адреса
              
              if (pWire->endTransmission() == 0) {
                pWire->requestFrom(address, 1);
                if (pWire->available()) {
                  uint8_t testByte = pWire->read();
                  
                  // Если байт совпал с нулевой ячейкой, проверяем инверсией (чтобы исключить случайное совпадение)
                  if (testByte == byte0) {
                    // Временно пишем инвертированное значение в ячейку 0
                    pWire->beginTransmission(address);
                    pWire->write(0x00); pWire->write(0x00);
                    pWire->write((uint8_t)~byte0);
                    pWire->endTransmission();
                    delay(5); // Ждем окончания физической записи в EEPROM (макс. 5мс)

                    // Снова читаем тестовый адрес
                    pWire->beginTransmission(address);
                    pWire->write((uint8_t)(testAddr >> 8));
                    pWire->write((uint8_t)(testAddr & 0xFF));
                    pWire->endTransmission();
                    pWire->requestFrom(address, 1);
                    
                    bool isMirrored = (pWire->available() && pWire->read() == (uint8_t)~byte0);

                    // Возвращаем исходный байт назад в ячейку 0
                    pWire->beginTransmission(address);
                    pWire->write(0x00); pWire->write(0x00);
                    pWire->write(byte0);
                    pWire->endTransmission();
                    delay(5);

                    if (isMirrored) {
                      kbits = sizesToTest[i]; // Объем определен!
                      break;
                    }
                  }
                }
              }
            }
            if (LOGGING) {
              // Выводим результат сканирования в консоль
              Serial.print(" - это EEPROM AT24C");
              Serial.print(kbits);
              Serial.print(" (Объем: ");
              Serial.print(kbits / 8); // Переводим килобиты в килобайты (32 Кбит = 4 Кбайт)
              Serial.print(" Кбайт)");
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

      if (address == 0x58) {
        if (LOGGING) {Serial.print(" - это зеркальный адрес (страница) памяти EEPROM AT24Cxx");}
      }


      // для дисплея LCD1602 / LCD2004
      if (address == 0x27) {
        if (LOGGING) { Serial.print(" - найден дисплей на 0x27. Инициализация..."); }
        // 1. Сохраняем базовые настройки в системный массив маяка
        device_DL[0] = 1;
        device_DL[1] = WIRE_NO;
        device_DL[2] = PIN_SDA;
        device_DL[3] = PIN_SCL;
        device_DL[4] = address;
        device_DL_name = "H44780 LCD Дисплей";
        // 2. Полностью останавливаем аппаратный I2C для защиты от зависаний
        pWire->end();
        LCD_init(true);
        LCD_print("I2C Scan: OK!", 0, 0);
        LCD_print("Display Active", 1, 0);
        // 5. ВОЗВРАЩАЕМ АППАРАТНЫЙ I2C НАЗАД для остальных датчиков маяка
        pWire->begin();
        pWire->setClock(400000);
        if (LOGGING) { Serial.println(" ок"); }
      }


      // для дисплея OLED 0.96"
      if (address == 0x3C) {
        if (LOGGING) {Serial.print(" - это дисплей OLED 0.96'");}
      }

      // для дисплея OLED 0.96"
      if (address == 0x3D) {
        if (LOGGING) {Serial.print(" - это дисплей OLED 0.96'");}
      }


      // для датчика давления BME280/BMP280
      if (address == 0x76) {
        if (LOGGING) {Serial.print(" - это датчик давления ");}
        uint8_t chipID = 0;
        pWire->beginTransmission(address);
        pWire->write(0xD0);
        pWire->endTransmission();
        pWire->requestFrom(address, 1);
        chipID = pWire->read();
        if (chipID == 0x60) {
          if (LOGGING) {Serial.print("BME280");}
          device_BM_name = "BME280 Гигрометр";
        } 
        else if (chipID == 0x58 || chipID == 0x56 || chipID == 0x57) {
          if (LOGGING) {Serial.print("BMP280");}
          device_BM_name = "BMP280 Барометр";
        } 
        else {
          if (LOGGING) {Serial.print("неизвестный чип");}
          device_BM_name = "неизвестный чип";
        }
        device_BM[0] = 1;
        device_BM[1] = WIRE_NO;
        device_BM[2] = PIN_SDA;
        device_BM[3] = PIN_SCL;
        device_BM[4] = address;
      }

      if (LOGGING) {Serial.println("");}
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("[Система] Неизвестная ошибка по адресу: 0x");
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

