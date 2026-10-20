/**  
 * ============================================================================  
 *  rtty_modem.cpp — Модулятор радиотелетайпа RTTY кодом МТК-2 (Бодо)
 *  Версия 2.12 от 2026-10-01, автор RU0AOG
 * ============================================================================ 
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Формирует и передаёт в эфир текстовые сообщения телетайпным режимом RTTY  
 *  с частотной манипуляцией (FSK): два тона MARK (лог. «1») и SPACE (лог. «0»).  
 *  Кодировка — 5-битный код МТК-2 (Baudot/ITA2) с поддержкой трёх регистров:  
 *  латиница (LAT), цифры/знаки (FIG) и кириллица (RUS).  
 *  
 *  ТРЕХРЕГИСТРОВОЕ КОДИРОВАНИЕ МТК-2  
 *  ---------------------------------  
 *  Использует 5-битную структуру кодирования Бодо/ИTA-2 с поддержкой трех  
 *  языковых регистров: латиница (table_lat), цифры/знаки (table_fig) и кириллица  
 *  (table_rus). При смене типа символа в эфир автоматически выстреливается  
 *  управляющий код переключения регистра (LAT=0x1F, FIG=0x1B, RUS=0x00).  
 *  Каждый символ обрамляется одним СТАРТ-битом (SPACE) и полуторным СТОП-битом (MARK).  
 *  
 *  ВЧ-ФОРМИРОВАНИЕ И ФЛАГ ИНВЕРСИИ  
 *  -------------------------------  
 *  - Аппаратный путь (Si5351): частоты MARK и SPACE рассчитываются заранее  
 *    в виде 8-байтных пакетов. В зависимости от флага invert, частота SPACE  
 *    устанавливается либо на shift_hz ниже MARK (нормальный режим), либо выше.  
 *  - Программный путь (PIO DDS): настраивает фазонепрерывную генерацию, передавая  
 *    в функцию vfo_hardware_init знаковую double-величину сдвига частоты.  
 *  
 *  ПРЕРЫВАНИЕ И ЦИКЛ ОЖИДАНИЯ  
 *  --------------------------  
 *  Внутри send_rtty_bit крутится прецизионный цикл ожидания длины бита на micros().  
 *  Для защиты от зависания при случайных наводках на UART, опрос кнопок и ПК  
 *  выполняется короткими чанками через delayMicroseconds(2000) только во время  
 *  передачи длинного стоп-бита.  
 *  
 *  СТРУКТУРА КАДРА  
 *  ---------------  
 *  Каждый символ (send_rtty_code): СТАРТ-бит (SPACE) -> 5 бит данных (LSB  
 *  первым) -> СТОП-бит (MARK) с полуторной стоп-паузой. При смене регистра  
 *  сначала отправляется управляющий код LAT/FIG/RUS. Длительность бита —  
 *  RTTY_BIT_TIME_US (по умолчанию ~22000 мкс = 45.45 Бод).  
 *  send_rtty_string оборачивает текст пилот-тоном MARK 500 мс (преамбула) и  
 *  хвостом MARK 500 мс (постамбула).  
 *  
 *  ТАБЛИЦЫ КОДИРОВКИ  
 *  -----------------  
 *  table_lat / table_fig / table_rus (в PROGMEM) сопоставляют символы (в т.ч.  
 *  многобайтовые UTF-8 кириллицы) 5-битным кодам МТК-2. find_in_table ищет  
 *  совпадение и дублирует символ в Serial и на LCD.  
 *  
 *  ПУБЛИЧНЫЙ API  
 *  -------------  
 *  prepare_rtty_frequencies(space_hz, mark_hz) — подготовка сетки частот.  
 *  send_rtty_string(String / const char*)      — передача текста с преамбулой/  
 *                                                постамбулой.  
 *  
 *  ИСПОЛЬЗУЕМЫЕ БИБЛИОТЕКИ И МОДУЛИ  
 *  -------------------------------  
 *  - Raspberry Pi Pico SDK (BSD-3): hardware/watchdog.h (watchdog_update)  
 *  - Arduino core RP2040 (LGPL-2.1): Serial, micros(), delayMicroseconds(),  
 *    snprintf, PROGMEM  
 *  - Модули проекта: rtty_modem.h, si5351_driver.h, file_manager.h,  
 *    lcd.h, led_blink.h, vfo_hardware.h  
 *  - Сторонний код отсутствует: кодирование МТК-2/ITA2 — общедоступный  
 *    стандарт, реализация и таблицы регистров LAT/FIG/RUS — оригинальная  
 *    разработка автора проекта.  
 *  
 *  Этот файл — часть RP2040_beacon, распространяется под GPLv3  
 *  (см. LICENSE в корне репозитория). 
 * ============================================================================  
 */

#include "rtty_modem.h"
#include "si5351_driver.h" 
#include "file_manager.h"  
#include "lcd.h"
#include "led_blink.h"
#include "vfo_hardware.h"
#include "thor_modem.h"
#include <hardware/watchdog.h>

// ДИАГНОСТИКА
#define RTTY_DEBUG              0         // 0 — выключить отладочный вывод тонов

// Внешние ссылки на глобальные переменные управления
extern bool soft_restart_flag;
extern volatile bool pc_file_written;
extern void check_serial_commands();   // Ссылка на проверку команд из Serial
extern void si_calculate_freq_bytes_mHz(uint64_t freq_mHz, uint8_t* out_data);
extern volatile uint32_t RTTY_BIT_TIME_US;

static MTK2_STATE current_reg = LAT;

#define MTK2_LAT 0x1F 
#define MTK2_FIG 0x1B 
#define MTK2_RUS 0x00

// Таблицы кодировки МТК-2
const mtk2_map_t table_lat[] PROGMEM = {
    {"A", 0x03}, {"B", 0x19}, {"C", 0x0E}, {"D", 0x09}, {"E", 0x01}, 
    {"F", 0x0D}, {"G", 0x1A}, {"H", 0x14}, {"I", 0x06}, {"J", 0x0B}, 
    {"K", 0x0F}, {"L", 0x12}, {"M", 0x1C}, {"N", 0x0C}, {"O", 0x18}, 
    {"P", 0x16}, {"Q", 0x17}, {"R", 0x0A}, {"S", 0x05}, {"T", 0x10}, 
    {"U", 0x07}, {"V", 0x1E}, {"W", 0x13}, {"X", 0x1D}, {"Y", 0x15}, {"Z", 0x11},
    {"a", 0x03}, {"b", 0x19}, {"c", 0x0E}, {"d", 0x09}, {"e", 0x01}, 
    {"f", 0x0D}, {"g", 0x1A}, {"h", 0x14}, {"i", 0x06}, {"j", 0x0B}, 
    {"k", 0x0F}, {"l", 0x12}, {"m", 0x1C}, {"n", 0x0C}, {"o", 0x18}, 
    {"p", 0x16}, {"q", 0x17}, {"r", 0x0A}, {"s", 0x05}, {"t", 0x10}, 
    {"u", 0x07}, {"v", 0x1E}, {"w", 0x13}, {"x", 0x1D}, {"y", 0x15}, {"z", 0x11},
    {" ", 0x04}, {"\n", 0x08}, {"\r", 0x02}, {NULL, 0}
};

const mtk2_map_t table_fig[] PROGMEM = {
    {"-", 0x03}, {"?", 0x19}, {":", 0x0E}, {"3", 0x01}, {"Э", 0x0D},
    {"Ш", 0x1A}, {"Щ", 0x14}, {"8", 0x06}, {"Ю", 0x0B}, {"(", 0x0F},
    {")", 0x12}, {".", 0x1C}, {",", 0x0C}, {"9", 0x18}, {"0", 0x16},
    {"1", 0x17}, {"4", 0x0A}, {"'", 0x05}, {"5", 0x10}, {"7", 0x07},
    {"=", 0x1E}, {"2", 0x13}, {"/", 0x1D}, {"6", 0x15}, {"+", 0x11},
    {"э", 0x0D}, {"ш", 0x1A}, {"щ", 0x14}, {"ю", 0x0B}, {"Ч", 0x0A}, {"ч", 0x0A},
    {NULL, 0}
};

const mtk2_map_t table_rus[] PROGMEM = {
    {"А", 0x03}, {"Б", 0x19}, {"Ц", 0x0E}, {"Д", 0x09}, {"Е", 0x01}, 
    {"Ф", 0x0D}, {"Г", 0x1A}, {"Х", 0x14}, {"И", 0x06}, {"Й", 0x0B}, 
    {"К", 0x0F}, {"Л", 0x12}, {"М", 0x1C}, {"Н", 0x0C}, {"О", 0x18}, 
    {"П", 0x16}, {"Я", 0x17}, {"Р", 0x0A}, {"С", 0x05}, {"Т", 0x10}, 
    {"У", 0x07}, {"Ж", 0x1E}, {"В", 0x13}, {"Ь", 0x1D}, {"Ы", 0x15}, 
    {"З", 0x11}, {"Ъ", 0x1D}, {"Ё", 0x01},
    {"а", 0x03}, {"б", 0x19}, {"ц", 0x0E}, {"д", 0x09}, {"е", 0x01}, 
    {"ф", 0x0D}, {"г", 0x1A}, {"х", 0x14}, {"и", 0x06}, {"й", 0x0B}, 
    {"к", 0x0F}, {"л", 0x12}, {"м", 0x1C}, {"н", 0x0C}, {"о", 0x18}, 
    {"п", 0x16}, {"я", 0x17}, {"р", 0x0A}, {"с", 0x05}, {"т", 0x10}, 
    {"у", 0x07}, {"ж", 0x1E}, {"в", 0x13}, {"ь", 0x1D}, {"ы", 0x15}, 
    {"з", 0x11}, {"ъ", 0x1D}, {"ё", 0x01},
    {NULL, 0}
};

// Буферы частот RTTY (8 байт частоты)
static uint8_t rtty_reg_space[8];
static uint8_t rtty_reg_mark[8];
static bool rtty_inverted = false;   // флаг инверсии MARK/SPACE, запоминается в prepare

// Внутренняя функция поиска совпадений по таблицам
static bool find_in_table(const mtk2_map_t* table, const char* s, uint8_t &code, int &len) {
    for (int i = 0; table[i].s != NULL; i++) {
        int l = strlen(table[i].s);
        if (strncmp(table[i].s, s, l) == 0) {
            code = table[i].code;
            len = l;
            if (l > 0 && table[i].s[0] != '\r' && table[i].s[0] != '\n') { // не печатать в командной строке перевод каретки
                Serial.print(table[i].s);
                LCD_push_char(table[i].s, 0, 16);
            }
            Serial.flush();
            return true;
        }
    }
    return false;
}

// Подготовка сетки частот на основе MARK, SHIFT и флага INVERT
void prepare_rtty_frequencies(uint32_t rtty_freq, uint32_t shift_hz, bool invert) {
  rtty_inverted = invert;
  static uint32_t rtty_mark_hz = 0, rtty_shift_hz = 0;   // для восстановления сетки после RSID
  uint32_t mark_hz;
  uint32_t space_hz;
  // Расчет частоты SPACE на основе флага инверсии
  if (invert) {
    space_hz = rtty_freq - (shift_hz + 1) / 2; // Инверсный режим: Space выше Mark
    mark_hz = space_hz - shift_hz;
  } else {
    mark_hz = rtty_freq + (shift_hz + 1) / 2; // Нормальный режим: Mark выше Space
    space_hz = mark_hz - shift_hz;
  }
  rtty_mark_hz = mark_hz;  
  rtty_shift_hz = shift_hz;   // знак сдвига уже учтён при расчёте space

  if (device_SI[0]) {
    // аппаратный генератор
    uint64_t space_mHz = (uint64_t)space_hz * 1000ULL;
    uint64_t mark_mHz  = (uint64_t)mark_hz * 1000ULL;
    si_calculate_freq_bytes_mHz(space_mHz, rtty_reg_space); // считаем таблицу байт частоты для СПЕЙС
    si_calculate_freq_bytes_mHz(mark_mHz,  rtty_reg_mark);  // считаем таблицу байт частоты для МАРК
    if (debug_flag) {
        Serial.println(F("[RTTY_ГОТОВ] Сетка частот RTTY для Si5351 готова.")); 
        Serial.print(F("            F_MARK : ")); Serial.print(mark_hz); Serial.println(F(" Hz"));
        Serial.print(F("            F_SPACE: ")); Serial.print(space_hz); Serial.println(F(" Hz"));
    }
  }
  else {
    // Для программного VFO RP2040
    vfo_clk_boost_enter(0);   // активация режима разгона
    vfo_set_narrowband(true);
    // RSID-маркер режима до пилот-тона: fldigi переключит декодер на RTTY.  
    // Центр полосы RTTY = mark - shift/2.  
    rsid_send(39 /*RSID_RTTY45*/, (double)rtty_mark_hz - (double)rtty_shift_hz / 2.0);  
    // rsid_send перенастроила VFO на свою сетку — вернуть RTTY-сетку  
    //vfo_set_narrowband(true);  // mfsk_prepare мог сбросить narrowband  
    // Сетка растёт вверх от базы: base = min(mark, space), step = shift.  
    // Нормальный режим: tone0=SPACE, tone1=MARK. Инверсия — наоборот.  
    uint32_t base_hz = invert ? mark_hz : space_hz;  
    vfo_hardware_init(base_hz, shift_hz);  
    vfo_set_tone_instant(invert ? 0 : 1); // стартуем с MARK  
    //vfo_operation_set(true);    // запустить генерацию программного VFO
    if (debug_flag) {
        Serial.println(F("[RTTY_ГОТОВ] Программный DDS VFO инициализирован!"));
        Serial.print(F("            MARK :  ")); Serial.print(format_freq(mark_hz)); Serial.println(F(" Гц"));
        Serial.print(F("            SPACE:  ")); Serial.print(format_freq(space_hz)); Serial.println(F(" Гц"));
    }
  }
}


// Передача бита на чип Si5351 (8-байтный пакет + команда старта)
static void send_rtty_bit(TransmitterState state) {
    if (pc_file_written || soft_restart_flag) return;

#if RTTY_DEBUG 
        while(1) {
        watchdog_update();
        vfo_set_tone_instant(0);
        ZERO_LED_RED_ON();
        delay(1000);
        vfo_set_tone_instant(1);
        ZERO_LED_GREEN_ON();
        delay(1000);
        }
#endif 

      if (device_SI[0]) {
        uint8_t* data = (state == MARK) ? rtty_reg_mark : rtty_reg_space;
        setFrq_si5351(data); // установить частоту
        VFO_TX_ON();            // запустить выход частоты TX
      }
      else {
        // Сетка: норма tone0=SPACE/tone1=MARK; инверсия — зеркально.  
        uint8_t tone = (state == MARK) ? (rtty_inverted ? 0 : 1)  
                                       : (rtty_inverted ? 1 : 0);  
        vfo_set_tone_instant(tone);
      }

    if (state == MARK) {
        ZERO_LED_RED_ON();
        // while(1) {watchdog_update();} // марк-холд
    } else {
        ZERO_LED_GREEN_ON();
        // while(1) {watchdog_update();} // спейс-холд
    }

    // Засекаем точное время начала передачи бита
    uint32_t start_bit_us = micros();
    // Крутим точный цикл ожидания длительности бита
    while (micros() - start_bit_us < RTTY_BIT_TIME_US) {
        // Быстрая проверка: прилетели ли данные в UART?
        // Мы НЕ вызываем тяжелый парсер check_serial_commands()!
        if (Serial.available() > 0) {
            // Если пользователь что-то нажал в терминале во время передачи — 
            // мы расцениваем это как запрос на экстренную остановку (Break)
            soft_restart_flag = true; 
            break;
        }
        // Даем процессору RP2040 слегка «подышать» (опционально)
        delayMicroseconds(10);
        watchdog_update(); // сброс сторожевого таймера в процессе передачи
    }
}





// Отправка 5-битной посылки кода Бодо/МТК-2
static void send_rtty_code(uint8_t code) {
    if (pc_file_written || soft_restart_flag) return; 

    send_rtty_bit(SPACE); // СТАРТ-БИТ
    
    for (int i = 0; i < 5; i++) {
        if (pc_file_written || soft_restart_flag) return;
        if (code & (1 << i)) send_rtty_bit(MARK);
        else send_rtty_bit(SPACE);
    }
    
    if (pc_file_written || soft_restart_flag) return;
    send_rtty_bit(MARK); // СТОП-БИТ
    
    // Полуторный стоп-бит
    uint32_t stop_pause = RTTY_BIT_TIME_US / 2;
    uint32_t stop_chunks = stop_pause / 2000;
    for (uint32_t i = 0; i < stop_chunks; i++) {
        if (pc_file_written || soft_restart_flag) return;
        check_serial_commands();
        delayMicroseconds(2000);
    }
}

// Главная функция передачи текста кодом МТК-2
void send_rtty_raw(const char* s) {
    // Начинаем всегда с латинского регистра
    current_reg = LAT;
    send_rtty_code(MTK2_LAT);

    while (*s) {
        if (pc_file_written || soft_restart_flag) break; 
        uint8_t code = 0;
        int char_len = 1;
        bool found = false;

        // Шаг 1. Сначала ищем универсальные символы (пробел, перевод строки) в текущем регистре,
        // чтобы предотвратить паразитную отправку кодов смены языка.
        if (*s == ' ')        { code = 0x04; char_len = 1; found = true; Serial.print(F(" ")); LCD_push_char(" ", 0, 16);}
        else if (*s == '\n')  { code = 0x08; char_len = 1; found = true; LCD_push_char(" ", 0, 16);}
        else if (*s == '\r')  { code = 0x02; char_len = 1; found = true; LCD_push_char(" ", 0, 16);}
        
        // Шаг 2. Если это обычный символ — ищем по языковым таблицам
        if (!found) {
            MTK2_STATE target_reg = current_reg;
            
            if (find_in_table(table_lat, s, code, char_len)) { target_reg = LAT; found = true; }
            else if (find_in_table(table_fig, s, code, char_len)) { target_reg = FIG; found = true; }
            else if (find_in_table(table_rus, s, code, char_len)) { target_reg = RUS; found = true; }

            if (found) {
                // Если символ требует смены регистра букв/цифр
                if (target_reg != current_reg) {
                    uint8_t reg_code = (target_reg == LAT) ? MTK2_LAT : (target_reg == FIG ? MTK2_FIG : MTK2_RUS);
                    send_rtty_code(reg_code);
                    current_reg = target_reg;
                }
            }
        }
        
        // Шаг 3. Физическая отправка кода символа в эфир
        if (found) {
            send_rtty_code(code);
            s += char_len;  // Сдвигаем указатель на длину символа (1 байт для ASCII, 2 для UTF-8 кириллицы)
        } else {
            s++;  // Пропускаем неизвестный символ
        }
    }
    // ФИНАЛ СТРОКИ: отправляем служебные CR/LF
    if (!pc_file_written && !soft_restart_flag) {
        send_rtty_code(0x02); // CR
        send_rtty_code(0x08); // LF
    }
    ZERO_LED_OFF();
    Serial.println("");
}



// функция со String, включающая преамбулу, текст и постамбулу
void send_rtty_string(String str) {
  if (pc_file_written == false && soft_restart_flag == false) {

    vfo_set_narrowband(true);
    if (!device_SI[0]) {  
        vfo_operation_set(false);  
        vfo_set_tone_instant(rtty_inverted ? 0 : 1); // стартовый тон = MARK
        vfo_operation_set(true);   // ← обратное включение выхода  
    }
    // СТАРТОВЫЙ ПИЛОТ-ТОН (PREAMBLE): Включаем частоту MARK на 500 мс
    uint32_t preamble_start = millis();
    while (millis() - preamble_start < 500) {
        if (pc_file_written || soft_restart_flag) break; 
        send_rtty_bit(MARK); 
    }
    // ОСНОВНАЯ ПЕРЕДАЧА текста
    if (!pc_file_written && !soft_restart_flag) {
        send_rtty_raw(str.c_str()); 
    }
    // ФИНАЛЬНЫЙ ХВОСТ (POSTAMBLE): Удерживаем частоту MARK еще 500 мс
    uint32_t postamble_start = millis();
    while (millis() - postamble_start < 500) {
        if (pc_file_written || soft_restart_flag) break; 
        send_rtty_bit(MARK);
    }
    // Закрываем сессию связи
    if (device_SI[0]) {
        VFO_TX_OFF(); 
    } else {
        vfo_operation_set(false);
    }
    ZERO_LED_OFF();
    vfo_set_narrowband(false);   // возвращаем штатный дизер
  }
}


// функция-диспетчер
void send_rtty_string(const char* s) {
    send_rtty_string(String(s));
}
