#include "file_manager.h"
#include "si5351_driver.h"
#include "ifkp_modem.h"
#include "rtty_modem.h"  
#include "scheduler.h"
#include "cw_modem.h"
#include <hardware/watchdog.h> 
#include <Adafruit_TinyUSB.h>

// автоматический маяк на RP2040
// версия 2.02 от 2026-07-29, автор RU0AOG
// моды CW, RTTY, IFKP
// сканирование подключенного оборудования
// дисковая система с файлом конфигурации
// редактирование конфигурации из консоли
// работа по расписанию
// передача данных телеметрии

#define BCN_VER 2.02

bool force_cw_transmission   = false; // Флаг ручного запуска CW
bool force_rtty_transmission = false; // Флаг ручного запуска RTTY
bool force_ifkp_transmission = false; // Флаг ручного запуска IFKP

bool soft_restart_flag = false;
extern bool pc_activity_detected; 
bool is_transmitting = false; // Флаг передачи 
String rtc_chip_name = "Неизвестный RTC"; // Сюда сканер запишет точное имя чипа

extern uint8_t rtc_sec;

// Функция проверки и обработки текстовых команд с локальным эхом
void check_serial_commands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); 
    command.trim(); 

    if (command.length() == 0) return;

    Serial.print("\r\n> "); 
    Serial.println(command);

    if (command == "?") {
      // ОПРОС МАЯКА ВО ВРЕМЯ РАБОТЫ
      print_current_settings();
      print_current_date();
      print_current_time();
      Serial.print("Телеметрия: ");
      Serial.println(get_telemetry_string());
      Serial.println("Введите help для перехода в справочное меню по командам управления");
      Serial.println("");
      return;
    }
    // =========================================================================
    // КОМАНДА ТЕКУЩЕГО ВРЕМЕНИ И НАСТРОЙКИ
    // =========================================================================
    else if (command.startsWith("time")) {
      // Проверяем, есть ли что-то после слова "time"
      int space_idx = command.indexOf(' ');
      if (space_idx == -1) {
        // Если пробела нет (ввели строго "time" без аргументов) — это ЗАПРОС
        print_current_date();
        print_current_time();
        return;
      } else {
        // Если пробел есть — это УСТАНОВКА времени
        handle_time_command(command);
        return;
      }
    }
    else if (command.startsWith("date")) {
      handle_date_command(command);
      return;
    }
    else if (command == "restart") {
      Serial.println("[!] МЯГКИЙ ПЕРЕЗАПУСК МАЯКА...");
      Serial.flush();
      pc_file_written = true; 
      soft_restart_flag = true;
      return;
    }
    else if (command == "reset") {
      Serial.println("[!] КРИТИЧЕСКИЙ ЖЕСТКИЙ СБРОС ПРОЦЕССОРА...");
      Serial.flush();
      SI_POWER_OFF();
      delay(500);
      watchdog_reboot(0, 0, 0);
      return;
    }
    // =========================================================================
    // КОМАНДА ДЛЯ ТЕКСТА ПЕРЕДАЧИ (ПРОСМОТР И УСТАНОВКА)
    // =========================================================================
    if (command.startsWith("text")) {
      int space_idx = command.indexOf(' ');
      
      if (space_idx == -1) {
        // Если пробела нет (ввели строго "text" без аргументов) — это ЗАПРОС
        Serial.println(F("=== ТЕКУЩИЙ ТЕКСТ РАДИОПЕРЕДАЧИ МАЯКА ==="));
        Serial.print(F("Текст: ")); 
        Serial.println(my_text_variable.length() > 0 ? my_text_variable : F("Не задан"));
        Serial.println(F("========================================="));
        return;
      } else {
        // Если пробел есть — это УСТАНОВКА нового текста
        String new_txt = command.substring(space_idx + 1);
        new_txt.trim();
        
        if (new_txt.length() > 0) {
          extern void update_info_config_from_console(String marker, String new_value);
          update_info_config_from_console("TEXT", new_txt);
        } else {
          Serial.println(F("[Ошибка] Текст для записи пустой."));
        }
        return;
      }
    }
    // =========================================================================
    // КОМАНДЫ ПРИНУДИТЕЛЬНОГО РУЧНОГО СТАРТА ПЕРЕДАЧ
    // =========================================================================
    else if (command == "start cw") {
      if (is_transmitting) {
        Serial.println(F("[Ошибка] Сейчас уже идет трансляция! Дождитесь окончания сеанса."));
      } else {
        Serial.println(F("[Система] Заявка принята. Выходим в эфир CW на следующем такте..."));
        force_cw_transmission = true;
      }
      return;
    }
    else if (command == "start rtty") {
      if (is_transmitting) {
        Serial.println(F("[Ошибка] Сейчас уже идет трансляция! Дождитесь окончания сеанса."));
      } else {
        Serial.println(F("[Система] Заявка принята. Выходим в эфир RTTY на следующем такте..."));
        force_rtty_transmission = true;
      }
      return;
    }
    else if (command == "start ifkp") {
      if (is_transmitting) {
        Serial.println(F("[Ошибка] Сейчас уже идет трансляция! Дождитесь окончания сеанса."));
      } else {
        Serial.println(F("[Система] Заявка принята. Выходим в эфир IFKP на следующем такте..."));
        force_ifkp_transmission = true;
      }
      return;
    }


    else if (command.startsWith("setparam ")) {
      // Формат: setparam МАРКЕР=ЗНАЧЕНИЕ (например: setparam CALL=RU0AOG)
      String param_part = command.substring(9);
      param_part.trim();
      
      int eq_idx = param_part.indexOf('=');
      if (eq_idx != -1) {
        String marker = param_part.substring(0, eq_idx);
        String value  = param_part.substring(eq_idx + 1);
        marker.trim();
        value.trim();
        
        if (marker.length() > 0 && value.length() > 0) {
          extern void update_info_config_from_console(String marker, String new_value);
          update_info_config_from_console(marker, value);
        }
      return;
      } else {
        Serial.println("[Ошибка] Неверный формат. Используйте: setparam МАРКЕР=ЗНАЧЕНИЕ");
        return;
      }
    }
    else {
      Serial.println("=== СПРАВКА ПО КОМАНДАМ УПРАВЛЕНИЯ ===");
      Serial.println("help(или другое)- Вывести это справочное меню");
      Serial.println("?               - Показать текущую конфигурацию из INFO.txt, дату и время RTC");
      Serial.println("time            - Вывести текущее время и дату");
      Serial.println("time ЧЧ:ММ      - Установить время часов, например: time 12:45");
      Serial.println("time ЧЧ:ММ:CC   - Установить время часов, например: time 19:30:0)");
      Serial.println("date ДД.ММ.ГГГГ - Установить календарную дату, например: date 18.05.2026");
      Serial.println("text            - Показать текущий текст радиопередачи из конфигурации INFO.txt");
      Serial.println("text [текст]    - Записать новый текст передачи в файл конфигурации, например: text NEW TEXT");
      Serial.println("setparam M=V    - Изменить любой маркер в конфигурации, например: setparam CALL=RA3ABC или setparam QTH=NA56AV");
      Serial.println("start cw        - Немедленно запустить внеочередной сеанс CW");
      Serial.println("start rtty      - Немедленно запустить внеочередной сеанс RTTY");
      Serial.println("start ifkp      - Немедленно запустить внеочередной сеанс IFKP");
      Serial.println("restart         - Мягкий виртуальный перезапуск маяка");
      Serial.println("reset           - Жесткий аппаратный сброс процессора RP2040");
      Serial.println("=======================================");
    }
  }
}

void setup() {
  init_file_manager();  // инициализировать флэш-диск
  // ожидание коннекта - 3 сек
  Serial.begin(115200);

  // Serial1.begin(115200, SERIAL_8N1); // Запуск аппаратного UART0 на пинах GP0 (TX) и GP1 (RX)
  // Serial.println("[Система] Выходим в эфир...");  // это пойдет в USB
  // Serial1.println("[Система] Выходим в эфир..."); // это пойдет по физическому проводу TX

  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 3000)) {
      #if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
      TinyUSBDevice.task();
      #endif
  }
  Serial.println("");
  Serial.println("================================================================");
  Serial.println("[Система] Питание маяка включено.");
  Serial.println("[Система] Последовательное соединение восстановлено.");
  pinMode(LED_BUILTIN, OUTPUT);
  I2C1_Scanner(SI_POWER_PIN, SI_PIN_SDA, SI_PIN_SCL);
  I2C1_Scanner(DS_POWER_PIN, DS_PIN_SDA, DS_PIN_SCL);

  init_si5351_pins();  // инициализировать си5351
  init_scheduler();    // инициализировать дс3231

  Serial.println(F("\n================================================================"));
  Serial.print(F("  АВТОМАТИЧЕСКИЙ МАЯК ВЕР."));
  Serial.print(BCN_VER);
  Serial.println(F(" НА СТАРТЕ"));
  Serial.println(F("  Плата готова к работе."));
  Serial.println(F("  Введите help для выхода в справочное меню."));
  Serial.println(F("================================================================\n"));

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
    Serial.println("Инициализация параметров радиомаяка...");
    
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
  // 3. ПЕРИОДИЧЕСКИЙ ВЫВОД ВРЕМЕНИ (Каждые 60 секунд)
  // =========================================================================
/*
  // Выводим время строго раз в минуту (на 00-й секунде)
  // если сейчас нет активного эфира и мы эту секунду ещё не печатали
  if (rtc_sec % 60 == 0) {
    if (rtc_sec != last_printed_sec && !pc_activity_detected && !soft_restart_flag && !is_transmitting) {
      last_printed_sec = rtc_sec; // Записываем 0, блокируя повторный вывод на этой секунде
      print_current_time(); 
    }
  } else {
    last_printed_sec = 255; // На других секундах сбрасываем флаг
  }
*/


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
      SI_POWER_ON();
      if (SI_FAIL == false && !pc_file_written) {
        uint32_t ifkp_hz = strtoul(my_freq_ifkp_var.c_str(), NULL, 10);
        if (ifkp_hz == 0) ifkp_hz = 3601307; // Резервная частота по умолчанию (3601,307 кГц)
        // Вывод параметров IFKP
        prepare_ifkp_frequencies(ifkp_hz);
        // IFKP: передача стартовой лесенки
        Serial.print("[ЭФИР_IFKP] Синхр: ");
        for(int i = 0; i < 10; i++) {
            if (pc_file_written || soft_restart_flag) break; 
            send_delta(1); 
            Serial.print("."); 
        }
        Serial.println("ОК");
        // IFKP: передача позывного и локатора
        Serial.print("[ЭФИР_IFKP] Вызов: ");
        send_ifkp_string("\r\n\r\nVVV BEACON DE " + my_call_variable + "/B DE " + my_call_variable + "/B, QTH " + my_qth_variable + " " + my_qth_variable + " \r\n"); 
        // IFKP: передача лесенки
        Serial.print("[ЭФИР_IFKP] Синхр: ");
        si5351_write_reg(0x03, 0x00); // Открываем выходы генерации Si5351 - нужно ли?
        for(int i = 0; i < 10; i++) {
            if (pc_file_written || soft_restart_flag) break; 
            send_delta(1); 
            Serial.print("."); 
        }
        Serial.println("ОК");
        // IFKP: передача основного текста
        Serial.print("[ЭФИР_IFKP] Текст: ");
        send_ifkp_string(my_text_variable + "\r\n"); 
        // IFKP: передача телеметрии
        String telemetry = get_telemetry_string();
        Serial.print("[ЭФИР_IFKP] Телем: ");
        si5351_write_reg(0x03, 0x00); // Открываем выходы генерации Si5351
        send_ifkp_string(telemetry);
        // IFKP завершение передачи
        Serial.print("[ЭФИР_IFKP] Конец: ");
        send_ifkp_string("OVER.\r\n\r\n");
        // обновить время
        update_scheduler();
        char end_buf[128];
        snprintf(end_buf, sizeof(end_buf), "[Система] : %02d:%02d:%02d - Сеанс IFKP завершен.", rtc_hour, rtc_min, rtc_sec);
        Serial.println(end_buf);
      } 
      is_transmitting = false;
      SI_POWER_OFF();
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
      SI_POWER_ON();
      if (SI_FAIL == false && !pc_file_written) {
        uint32_t rtty_space_hz = strtoul(my_rtty_space_var.c_str(), NULL, 10);
        if (rtty_space_hz == 0) rtty_space_hz = 3601000; // Дефолтный Space (3601,000 кГц)
        uint32_t rtty_mark_hz  = strtoul(my_rtty_mark_var.c_str(), NULL, 10);
        if (rtty_mark_hz == 0)  rtty_mark_hz = 3601170;  // Дефолтный Mark (3601,170 кГц)
	      // Вывод параметров RTTY
        prepare_rtty_frequencies(rtty_space_hz, rtty_mark_hz);
        Serial.print("[Скорость]: "); Serial.print(1000000.0f / RTTY_BIT_TIME_US, 2); Serial.print(" БОД, ");
        Serial.print("длительность бита "); Serial.print(RTTY_BIT_TIME_US/1000); Serial.println(" мс");
        // RTTY передача позывного и локатора
        Serial.print("[ЭФИР_RTTY] Вызов: ");
        send_rtty_string("\r\n\r\nVVV BEACON DE " + my_call_variable + "/B DE " + my_call_variable + "/B, QTH " + my_qth_variable + " " + my_qth_variable + " \r\n");
        // RTTY передача основного текста
        Serial.print("[ЭФИР_RTTY] Текст: ");
        send_rtty_string(my_text_variable + " ");
        // RTTY передача телеметрии
        String telemetry = get_telemetry_string();
        Serial.print("[ЭФИР_RTTY] Телем: ");
        send_rtty_string(telemetry);
        // RTTY завершение передачи
        Serial.print("[ЭФИР_RTTY] Конец: ");
        send_rtty_string("OVER.\r\n\r\n");
        // обновить время
        update_scheduler();
        char end_buf[128];
        snprintf(end_buf, sizeof(end_buf), "[Система] : %02d:%02d:%02d - Сеанс RTTY завершен.", rtc_hour, rtc_min, rtc_sec);
        Serial.println(end_buf);
      } 
      is_transmitting = false;
      SI_POWER_OFF();
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
      SI_POWER_ON();
      if (SI_FAIL == false && !pc_file_written) {
        uint32_t cw_hz = (my_freq_cw_var.length() > 0) ? strtoul(my_freq_cw_var.c_str(), NULL, 10) : 3601000;
	// Вывод параметров CW
        prepare_cw_frequency(cw_hz);
        Serial.print("[Скорость]: "); Serial.print(1200/CW_DOT_TIME_MS); Serial.print(" WPM, ");
        Serial.print("длительность точки "); Serial.print(CW_DOT_TIME_MS); Serial.println(" мс");
        // CW передача позывного и локатора
        Serial.print("[ЭФИР_CW] Вызов: ");
        send_cw_string("VVV BEACON DE " + my_call_variable + "/B DE " + my_call_variable + "/B, QTH " + my_qth_variable + " " + my_qth_variable + " ");
        // CW передача основного текста
        Serial.print("[ЭФИР_CW] Текст: ");
        send_cw_string(my_text_variable + " ");
        // CW передача телеметрии
        String telemetry = get_telemetry_string();
        Serial.print("[ЭФИР_CW] Телем: ");
        send_cw_string(telemetry); 
        // CW завершение передачи
        Serial.print("[ЭФИР_CW] Конец: ");
        send_cw_string("OVER.");
        // обновить время
        update_scheduler();
        char end_buf[128];
        snprintf(end_buf, sizeof(end_buf), "[Система] : %02d:%02d:%02d - Сеанс CW завершен.", rtc_hour, rtc_min, rtc_sec);
        Serial.println(end_buf);
      } 
      is_transmitting = false; 
      SI_POWER_OFF(); 
      Serial.println("");
    } 
  }

  delay(1); 
}


void I2C1_Scanner(int POWER_PIN, int PIN_SDA, int PIN_SCL) {
  // Настройка пинов и запуск шины I2C1
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH); 
  delay(200);
  Wire1.end();
  Wire1.setSDA(PIN_SDA);
  Wire1.setSCL(PIN_SCL);
  Wire1.begin();
  Wire1.setClock(400000);

  byte error, address;
  int nDevices = 0;

  Serial.print("[Система] Сканирование шины I2C-1 на пинах SDA="); Serial.print(PIN_SDA); Serial.print(", SCL="); Serial.print(PIN_SCL); Serial.println(":");

  for (address = 1; address < 127; address++) {
    // Начало передачи по адресу
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0) {
      Serial.print("[Система] - найден I2C прибор по адресу: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // для Si5351
      if (address == 0x60) {
        Wire1.beginTransmission(address);
        Wire1.write(0x00);
        if (Wire1.endTransmission() == 0) {
          Wire1.requestFrom(address, 1);
          if (Wire1.available()) {
            byte reg0 = Wire1.read();
            byte revid = reg0 & 0x03;
            Serial.print(" - это чип Si5351 ревизии ");
            Serial.print(revid);
          }
        }
      }

      // для DS3231/1307
      if (address == 0x68) {
        extern String rtc_chip_name;
        Wire1.beginTransmission(address);
        Wire1.write(0x11); // запрос регистра температуры ds3231
        if (Wire1.endTransmission() == 0) {
          Wire1.requestFrom(address, 1);
          if (Wire1.available()) {
            byte tempMSB = Wire1.read();
            if (tempMSB < 85 || tempMSB > 215) {
            Serial.print(" - это чип DS3231, температура ");
            Serial.print(tempMSB);
            Serial.println(" C");
            rtc_chip_name = "DS3231";
            } else {
              Serial.print(" - это чип DS1307");
              rtc_chip_name = "DS1307";
            }
          }
        }
      }

      // для флэш памяти AT24Cxx
      if (address == 0x50) {
        // ТЕСТОВОЕ ЧТЕНИЕ: проверяем отклик памяти
        Wire1.beginTransmission(address);
        Wire1.write(0x00); // отправляем старший байт адреса ячейки
        Wire1.write(0x00); // отправляем младший байт адреса ячейки
        if (Wire1.endTransmission() == 0) {
          Wire1.requestFrom(address, 1);
          if (Wire1.available()) {
            Wire1.read(); // читаем тестовый байт
            Serial.print(" - это чип флэш-памяти AT24Cxx");
          }
        }
      }

      if (address == 0x58) {
        Serial.print(" - это зеркальный адрес (страница) памяти EEPROM AT24Cxx");
      }

      // для дисплея LCD1602/1604
      if (address == 0x27) {
        Serial.print(" - это дисплей LCD1602/1604");
      }

      // для дисплея LCD1602/1604
      if (address == 0x3F) {
        Serial.print(" - это дисплей LCD1602/1604");
      }

      // для дисплея OLED 0.96"
      if (address == 0x3C) {
        Serial.print(" - это дисплей OLED 0.96'");
      }

      // для дисплея OLED 0.96"
      if (address == 0x3D) {
        Serial.print(" - это дисплей OLED 0.96'");
      }

      // для датчика давления BME280/BMP280
      if (address == 0x76) {
        Serial.print(" - это датчик давления BME280/BMP280");
      }

      // для датчика давления BME280/BMP280
      if (address == 0x77) {
        Serial.print(" - это датчик давления BME280/BMP280");
      }
      Serial.println("");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("[Система] Неизвестная ошибка по адресу: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.print("[Система] - I2C устройства не найдены.\n");
  }
  Serial.print("[Система] Сканирование завершено.\n");
  Wire1.end();
}