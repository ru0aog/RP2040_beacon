/**  
 * ============================================================================  
 *  ifkp_modem.cpp — Передатчик IFKP (инкрементальная частотная манипуляция)  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Формирует и передаёт в эфир текстовые сообщения цифровым режимом IFKP —  
 *  разновидностью MFSK, где символы кодируются НЕ абсолютным тоном, а РАЗНОСТЬЮ  
 *  (дельтой) между соседними тонами. Сетка — 33 тона, длительность знака 500 мс.  
 *  
 *  ПРИНЦИП IFKP (инкрементальное кодирование)  
 *  ------------------------------------------  
 *  Каждый символ передаётся сдвигом текущего индекса тона на дельту с переносом  
 *  по модулю 33: current_tone = (current_tone + delta) % 33 (см. send_delta).  
 *  Инкрементальность гарантирует, что каждый знак — это всегда СМЕНА тона,  
 *  что упрощает синхронизацию на приёме. Символ -> дельта задаётся в  
 *  send_ifkp_char; сложные символы шлются двумя дельтами (базовый код + модификатор  
 *  регистра/группы 29..31).  
 *  
 *  ДВА ВЧ-ТРАКТА (выбор по device_SI[0] / SI_FAIL)  
 *  -----------------------------------------------  
 *  1. Si5351 присутствует: 33 тона предрассчитываются в 8-байтные регистровые  
 *     пакеты (calculate_freq_bytes_mHz -> DATA_IFKP[33][8]) и переключаются через  
 *     set_ifkp_tone / setFrq_si5351 на CLK0. Совместимо с УКВ 144 МГц.  
 *  2. Si5351 отсутствует: используется программный VFO на PIO RP2040  
 *     (vfo_hardware_init с базой и шагом IFKP_STEP_HZ; переключение тонов  
 *     фазонепрерывно через vfo_set_tone_instant, вкл/выкл выхода — vfo_set_cw_key).  
 *  
 *  СЕТКА ЧАСТОТ  
 *  ------------  
 *  База IFKP_Base_freq (по умолчанию 3 601 307 Гц), шаг IFKP_STEP_HZ = 11.697 Гц  
 *  (= 386/33 Гц), 33 тона. prepare_ifkp_frequencies заполняет таблицу и печатает  
 *  базовую/среднюю частоты и ширину полосы.  
 *  
 *  КОДИРОВКА ТЕКСТА  
 *  ----------------  
 *  send_ifkp_string разбирает UTF-8 побайтно; двухбайтовая кириллица (префиксы  
 *  0xD0/0xD1) транслитерируется в ASCII-эквиваленты через trans_utf8_to_ifkp_char,  
 *  ASCII идёт напрямую. Затем каждый символ модулируется send_ifkp_char.  
 *  
 *  УПРАВЛЕНИЕ И ПРЕРЫВАНИЕ  
 *  -----------------------  
 *  Длительность тона выдерживается активным циклом на micros() с вызовом  
 *  check_serial_commands() (CLI не ломает тайминг). Передача мгновенно  
 *  прерывается по флагам pc_file_written / soft_restart_flag с гашением выхода  
 *  (CLK_OFF_si5351 или vfo_set_cw_key(false)). Индикация — LED_BUILTIN и  
 *  ZERO_LED_BLUE (гаснут на середине знака, эффект мигания).  
 *  
 *  СЕРВИС  
 *  ------  
 *  send_ifkp_calibration_ladder() — калибровочный свип-тест: последовательный  
 *  перебор всех 33 тонов по 500 мс для настройки/проверки сетки.  
 * ============================================================================  
 */

#include "ifkp_modem.h"
#include "si5351_driver.h"
#include "file_manager.h"
#include "scheduler.h"
#include "vfo_hardware.h"
#include "LED_BLINK.h"

uint32_t IFKP_Base_freq = 3601307;
// Глобальные переменные для хранения сетки частот IFKP (33 тона)
uint8_t DATA_IFKP[33][8];
uint8_t current_tone = 0; // Индекс текущего тона в сетке (0 - 32)
extern uint8_t device_SI[5]; // Экспорт флага сканера I2C устройств
const double IFKP_STEP_HZ = 11.697; // Задаем шаг вашей сетки (соответствует 386/33 Гц)



// Объявление функции расчета частот из соседнего модуля модема, чтобы вызвать её при включении питания
extern void prepare_ifkp_frequencies(uint32_t base_hz);
extern bool soft_restart_flag; 
extern volatile bool pc_file_written;
extern void check_serial_commands(); 
extern void calculate_freq_bytes_mHz(uint64_t freq_mHz, uint8_t* out_data);

// Запись рассчитанного тона в MultiSynth регистры Si5351
static void set_ifkp_tone(uint8_t tone_index) {
    if (tone_index > 32) return;
    uint8_t* data = DATA_IFKP[tone_index];
    setFrq_si5351(data, 0); // установить частоту для CLK0
}

// Передача разницы (дельта) частоты с микрозадержками и мгновенным выходом
void send_delta(uint8_t delta) {
    // Прерываем работу, если зафиксирована активность ПК или подан сигнал рестарта
    if (pc_file_written || soft_restart_flag) {
      digitalWrite(LED_BUILTIN, LOW);
      if (SI_FAIL == false) CLK_OFF_si5351(0); // Глушим синтезатор
      return;
    }
    
    current_tone = (current_tone + delta) % 33;

    // Установка тона в зависимости от доступного железа
    if (device_SI[0] && SI_FAIL == false) {
      set_ifkp_tone(current_tone);
    } else {
      vfo_set_tone_instant(current_tone); // Мгновенный фазонепрерывный прыжок в PIO
    }

    // Включаем светодиод индикации передачи
    digitalWrite(LED_BUILTIN, HIGH);
    ZERO_LED_BLUE_ON();
    uint32_t IFKP_start_time = micros();
    uint32_t IFKP_tone_duration_us = 500000; // 500 миллисекунд в микросекундах
    uint32_t IFKP_halftone_duration_us = IFKP_tone_duration_us / 2;
    bool led_half_turned_off = false;

    // Аппаратный цикл удержания длительности знака
    while (micros() - IFKP_start_time < IFKP_tone_duration_us) {
        // Прерываем работу, если зафиксирована активность ПК или подан сигнал рестарта
        if (pc_file_written || soft_restart_flag) {
            digitalWrite(LED_BUILTIN, LOW);
            ZERO_LED_OFF();
            if (device_SI[0] && SI_FAIL == false) CLK_OFF_si5351(0);
            else vfo_set_cw_key(false);
            return;
        }
        // Опрашиваем CLI. Любые задержки внутри CLI больше не ломают общую длительность тона
        check_serial_commands();
        
        // гасим светодиод на экваторе длительности (250 мс)
        if (!led_half_turned_off && (micros() - IFKP_start_time >= IFKP_halftone_duration_us)) {
            digitalWrite(LED_BUILTIN, LOW);
            ZERO_LED_OFF();
            led_half_turned_off = true;
        }
        yield(); // Разгрузка ядра процессора RP2040
    }
}


// Заполнение таблицы базовых частот DATA_IFKP (совместимо с УКВ 144 МГц)
void prepare_ifkp_frequencies(uint32_t base_hz) {
    current_tone = 0; // Сброс индекса перед началом сессии

    // ЕСЛИ СИНТЕЗАТОР ОТСУТСТВУЕТ - ИНИЦИАЛИЗИРУЕМ PIO ЯДРО И ВЫХОДИМ
    if (!device_SI[0] || SI_FAIL == true) {
        vfo_hardware_init(base_hz, IFKP_STEP_HZ);
        vfo_set_cw_key(true); // Открываем ВЧ-выход ноги 28 в эфир
        vfo_set_tone_instant(0);
        Serial.print(F("[IFKP_PIO] Аппаратное PIO-ядро готово. База: ")); Serial.print(base_hz);
        Serial.print(F(" Гц, Шаг: ")); Serial.print(IFKP_STEP_HZ, 3); Serial.println(F(" Гц"));
        return; 
    }

    // Родной код расчета байт для Si5351 (выполняется, только если чип на месте)
    uint64_t base_mHz = (uint64_t)base_hz * 1000ULL;
    Serial.println(F("[IFKP_СЕТКА] Расчет тонов и вывод основных частот IFKP:"));
    for (int i = 0; i < 33; i++) {
        // Шаг 386/33 = 11.697 Гц
        uint64_t freq_mHz = base_mHz + ((uint64_t)i * 11697ULL);
        calculate_freq_bytes_mHz(freq_mHz, DATA_IFKP[i]);
    }
    uint32_t base_hz_part = (uint32_t)(base_mHz / 1000ULL);
    uint32_t span_hz_part = (uint32_t)((((uint64_t)32 * 11697ULL)) / 1000ULL);
    uint32_t midd_hz_part = (uint32_t)(base_hz_part+(span_hz_part/2));

    Serial.print(F("Базовая частота: ")); Serial.print(make_freq_with_space(base_hz_part)); Serial.println(F(" Гц")); 
    Serial.print(F("Средняя частота: ")); Serial.print(make_freq_with_space(midd_hz_part)); Serial.println(F(" Гц")); 
    Serial.print(F("Ширина полосы  : ")); Serial.print(span_hz_part); Serial.println(F(" Гц")); 
    Serial.println(F("[IFKP_ГОТОВ] 33 тона сетки частот IFKP успешно рассчитаны."));
}

// Модулятор протокола IFKP
void send_ifkp_char(char c) {
    // ИСПРАВЛЕНО: Быстрая проверка перед началом отправки составного символа
    if (pc_file_written || soft_restart_flag) return;

    if (c != '\r' && c != '\n') {Serial.print(c);} // не печатать в командной строке перевод каретки

    if (c >= 'a' && c <= 'z') send_delta((c - 'a' + 1) + 1); 
    else if (c == '.')  { send_delta(27 + 1);}
    else if (c == ' ')  { send_delta(28 + 1);}
    else if (c >= 'A' && c <= 'Z') {
        send_delta((c - 'A' + 1) + 1); 
        if (!pc_file_written && !soft_restart_flag) send_delta(29 + 1); // ИСПРАВЛЕНО: Защита второго байта
    }
    else if (c >= '1' && c <= '9') {
        send_delta((c - '1' + 1) + 1); 
        if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); 
    }
    else if (c == '0') {
        send_delta(10 + 1); 
        if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); 
    }
    else if (c == '\n')  { send_delta(28 + 1); if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1);}
    else if (c == '@')  { send_delta(0 + 1);   if (!pc_file_written && !soft_restart_flag) send_delta(29 + 1); }
    else if (c == ',')  { send_delta(27 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(29 + 1); }
    else if (c == '?')  { send_delta(28 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(29 + 1); }
    else if (c == '~')  { send_delta(0 + 1);   if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '!')  { send_delta(11 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '"')  { send_delta(12 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '#')  { send_delta(13 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '$')  { send_delta(14 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '%')  { send_delta(15 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '&')  { send_delta(16 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '\'') { send_delta(17 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '(')  { send_delta(18 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == ')')  { send_delta(19 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '*')  { send_delta(20 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '+')  { send_delta(21 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '-')  { send_delta(22 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '/')  { send_delta(23 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == ':')  { send_delta(24 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == ';')  { send_delta(25 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '<')  { send_delta(26 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '>')  { send_delta(27 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(30 + 1); }
    else if (c == '=')  { send_delta(0 + 1);   if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '[')  { send_delta(1 + 1);   if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '\\') { send_delta(2 + 1);   if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == ']')  { 
        send_delta(3 + 1);  
        if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); 
        if (!pc_file_written && !soft_restart_flag) send_delta(4 + 1);  
        if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1);
    }
    else if (c == '_')  { send_delta(5 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '{')  { send_delta(6 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '|')  { send_delta(7 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '}')  { send_delta(8 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '`')  { send_delta(9 + 1);  if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '\xB1') { send_delta(10 + 1); if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '\xF7') { send_delta(11 + 1); if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '\xB0') { send_delta(12 + 1); if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '\xD7') { send_delta(13 + 1); if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '\xA3') { send_delta(14 + 1); if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }
    else if (c == '\x7F') { send_delta(28 + 1); if (!pc_file_written && !soft_restart_flag) send_delta(31 + 1); }

}

char trans_utf8_to_ifkp_char(uint16_t utf8_char) {
    // Если это чистый ASCII (0..127) — возвращаем как есть.
    if (utf8_char <= 0x7F) return (char)utf8_char;
    
    // Маппинг двухбайтовой кириллицы (ЗАГЛАВНЫЕ и строчные) в ASCII-эквиваленты IFKP
    switch (utf8_char) {
        // А, а
        case 0xD090: case 0xD0B0: return 'A'; 
        // Б, б
        case 0xD091: case 0xD0B1: return 'B'; 
        // В, в
        case 0xD092: case 0xD0B2: return 'W';
        // Г, г
        case 0xD093: case 0xD0B3: return 'G'; 
        // Д, д
        case 0xD094: case 0xD0B4: return 'D'; 
        // Е, е, Ё, ё
        case 0xD095: case 0xD0B5: 
        case 0xD081: case 0xD191: return 'E'; 
        // Ж, ж
        case 0xD096: case 0xD0B6: return 'V'; 
        // З, з
        case 0xD097: case 0xD0B7: return 'Z';
        // И, и
        case 0xD098: case 0xD0B8: return 'I'; 
        // Й, й
        case 0xD099: case 0xD0B9: return 'J'; 
        // К, к
        case 0xD09A: case 0xD0BA: return 'K';
        // Л, л
        case 0xD09B: case 0xD0BB: return 'L'; 
        // М, м
        case 0xD09C: case 0xD0BC: return 'M'; 
        // Н, н
        case 0xD09D: case 0xD0BD: return 'N';
        // О, о
        case 0xD09E: case 0xD0BE: return 'O'; 
        // П, п
        case 0xD09F: case 0xD0BF: return 'P'; 
        // Р, р
        case 0xD0A0: case 0xD180: return 'R';
        // С, с
        case 0xD0A1: case 0xD181: return 'S'; 
        // Т, т
        case 0xD0A2: case 0xD182: return 'T'; 
        // У, у
        case 0xD0A3: case 0xD183: return 'U';
        // Ф, ф
        case 0xD0A4: case 0xD184: return 'F'; 
        // Х, х
        case 0xD0A5: case 0xD185: return 'H'; 
        // Ц, ц
        case 0xD0A6: case 0xD186: return 'C';
        // Ч, ч
        case 0xD0A7: case 0xD187: return 'X'; 
        // Ш, ш
        case 0xD0A8: case 0xD188: return 'Q'; 
        // Щ, щ
        case 0xD0A9: case 0xD189: return 'Y'; 
        // Ы, ы
        case 0xD0AB: case 0xD18B: return 'Y'; 
        // Ь, ь, Ъ, ъ (обычно склеиваются)
        case 0xD0AC: case 0xD18C:
        case 0xD0AA: case 0xD18A: return 'X'; 
        // Э, э
        case 0xD0AD: case 0xD18D: return 'E'; 
        // Ю, ю
        case 0xD0AE: case 0xD18E: return 'U'; 
        // Я, я
        case 0xD0AF: case 0xD18F: return 'A'; 
        
        default:     return ' '; // Вся остальная экзотика глушится пробелом
    }
}


// Посимвольный перебор строки
void send_ifkp_string(const char* str) {
  if (str == nullptr) return;
  
  // Включаем физический выход ВЧ генерации
  if (device_SI[0] && SI_FAIL == false) {
    CLK_ON_si5351(0); 
  } else {
    vfo_set_cw_key(true);
  }
    
    int i = 0;
    while (str[i] != '\0') {
        // Выходим через break, чтобы гарантированно проскочить вниз к отключению Si5351
        if (pc_file_written || soft_restart_flag) break;

        uint8_t b1 = str[i];
        char clean_ascii = ' ';

        if ((b1 == 0xD0 || b1 == 0xD1) && str[i + 1] != '\0') {
            uint8_t b2 = str[++i];
            uint16_t combined = (b1 << 8) | b2;
            clean_ascii = trans_utf8_to_ifkp_char(combined);
        } else if (b1 <= 0x7F) {
            clean_ascii = trans_utf8_to_ifkp_char(b1);
        }
        
        // Вызываем модулятор знака
        send_ifkp_char(clean_ascii);

        // Проверяем флаги сразу после отправки символа
        if (pc_file_written || soft_restart_flag) break;
        i++;
    }

  // Выключаем физический выход ВЧ генерации (Уходим в Z-состояние / Паузу)
  if (device_SI[0] && SI_FAIL == false) {
    CLK_OFF_si5351(0); 
  } else {
    vfo_set_cw_key(false);
  }

    CLK_OFF_si5351(0); // Корректно тушим чип
    digitalWrite(LED_BUILTIN, LOW);
    ZERO_LED_OFF();
    Serial.println(""); 
}

// Функция-помощник для объектов String
void send_ifkp_string(String str) {
  send_ifkp_string(str.c_str());
}


// Передача калибровочной лесенки (Свип-тест от 0 до 32 тона с миганием)
void send_ifkp_calibration_ladder() {
    
    // Включаем физический ВЧ-выход
    if (device_SI && SI_FAIL == false) {
        CLK_ON_si5351(0); 
    } else {
        vfo_set_cw_key(true);
    }

    // Последовательный перебор всех тонов вверх
    for (uint8_t i = 0; i < 33; i++) {
        // Мгновенная проверка флагов ПК или аварийного рестарта во время свипа
        if (pc_file_written || soft_restart_flag) break;

        current_tone = i; // Жестко фиксируем индекс текущего тона

        // Физическое переключение частоты
        if (device_SI && SI_FAIL == false) {
            set_ifkp_tone(current_tone);
        } else {
            vfo_set_tone_instant(current_tone);
        }

        Serial.print(F(".")); 

        // Аппаратные флаги индикации начала тона
        digitalWrite(LED_BUILTIN, HIGH);
        ZERO_LED_BLUE_ON(); // Зажигаем красный индикатор на старте ступени
        bool led_half_turned_off = false;

        // Настройка временных меток для полупериода мигания (500 мс / 2 = 250 мс)
        uint32_t start_ms = millis();
        const uint32_t tone_duration_ms = 500;
        const uint32_t halftone_duration_ms = tone_duration_ms / 2;

        // Аппаратный цикл точного удержания длительности знака
        while (millis() - start_ms < tone_duration_ms) {
            if (pc_file_written || soft_restart_flag) {
                digitalWrite(LED_BUILTIN, LOW);
                ZERO_LED_OFF();
                if (device_SI && SI_FAIL == false) CLK_OFF_si5351(0); else vfo_set_cw_key(false);
                return;
            }
            check_serial_commands();
            
            // Гасим светодиоды ровно на середине длительности тона (через 250 мс)
            if (!led_half_turned_off && (millis() - start_ms >= halftone_duration_ms)) {
                digitalWrite(LED_BUILTIN, LOW);
                ZERO_LED_OFF(); // Выключаем индикацию, создавая эффект мигания
                led_half_turned_off = true;
            }
            yield(); // Разгрузка ядра процессора
        }
    }

    // Выключаем физический ВЧ-выход по завершении теста
    if (device_SI && SI_FAIL == false) {
        CLK_OFF_si5351(0); 
    } else {
        vfo_set_cw_key(false);
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    ZERO_LED_OFF();
}

