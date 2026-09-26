/**  
 * ============================================================================  
 *  cw_modem.cpp — Передатчик CW (телеграф, международный код Морзе)  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Формирует и передаёт в эфир текстовые сообщения телеграфным режимом CW —  
 *  манипуляцией несущей «вкл/выкл» (OOK) по международному коду Морзе.  
 *  Несущая одна (без сдвига частоты), знаки кодируются длительностью посылок.  
 *  
 *  ДВА ВЧ-ТРАКТА (выбор по наличию чипа device_SI[0])  
 *  --------------------------------------------------  
 *  1. Si5351 присутствует: несущая программируется 8-байтным пакетом  
 *     (calculate_freq_bytes_mHz -> setFrq_si5351 на CLK0), манипуляция ключом —  
 *     через CLK_ON_si5351 / CLK_OFF_si5351. Совместимо с УКВ 144 МГц.  
 *  2. Si5351 отсутствует: программный VFO на PIO RP2040 (vfo_hardware_init;  
 *     манипуляция ключом — vfo_set_cw_key, замыкающий/размыкающий ВЧ-выход  
 *     на GPIO 28). Переадресация ключа скрыта внутри CLK_ON/OFF_si5351.  
 *  
 *  ТАЙМИНГИ МОРЗЕ (кратно длине точки CW_DOT_TIME_MS)  
 *  --------------------------------------------------  
 *    точка (dit)          = 1 точка;  тире (dah) = 3 точки (send_cw_element);  
 *    пауза между посылками = 1 точка;  между буквами = 3 точки;  
 *    между словами        = 7 точек.  
 *  Скорость задаётся глобальной CW_DOT_TIME_MS (пересчитывается из WPM в  
 *  file_manager). cw_delay() держит паузы через micros() без накопления дрейфа  
 *  и параллельно опрашивает Serial (check_serial_commands) для экстренного  
 *  останова по флагам pc_file_written / soft_restart_flag.  
 *  
 *  ОБРАБОТКА ТЕКСТА (send_cw_string)  
 *  ---------------------------------  
 *  Символы приводятся к верхнему регистру; '\r' пропускается; '\n' заменяется  
 *  на разделитель '=' (код -...-); ' ' даёт межсловную паузу. Каждый знак  
 *  ищется в таблице Морзе morse_table (A-Z, 0-9, . , ? / - = !) и параллельно  
 *  дублируется в Serial и на LCD (LCD_push_char). Индикация посылок — светодиоды  
 *  LED_BUILTIN и ZERO_LED_RED.  
 *  
 *  СТРУКТУРЫ:  
 *    struct cw_map_t { char c; const char* morse; } — элемент таблицы Морзе  
 *        «символ -> строка из точек/тире».  
 *  
 *  API:  
 *    prepare_cw_frequency(freq_hz)     — установка частоты несущей (Si5351 или  
 *                                        программный VFO на PIO).  
 *    send_cw_string(const char*) / (String) — передача текста кодом Морзе.  
 * ============================================================================  
 */

#include "cw_modem.h"
#include "si5351_driver.h"
#include "scheduler.h"
#include "file_manager.h"
#include "lcd.h"
#include "vfo_hardware.h"
#include "led_blink.h"

extern bool soft_restart_flag;
extern volatile bool pc_file_written;
extern void check_serial_commands();
extern void calculate_freq_bytes_mHz(uint64_t freq_mHz, uint8_t* out_data);

static uint32_t cw_frequency_hz = 3601000;

// Структура для таблицы Морзе
struct cw_map_t {
    char c;
    const char* morse;
};

// Таблица международного кода Морзе
const cw_map_t morse_table[] = {
    {'A', ".-"},    {'B', "-..."},  {'C', "-.-."},  {'D', "-.."},   {'E', "."},
    {'F', "..-."},  {'G', "--."},   {'H', "...."},  {'I', ".."},    {'J', ".---"},
    {'K', "-.-"},   {'L', ".-.."},  {'M', "--"},    {'N', "-."},    {'O', "---"},
    {'P', ".--."},  {'Q', "--.-"},  {'R', ".-."},   {'S', "..."},   {'T', "-"},
    {'U', "..-"},   {'V', "...-"},  {'W', ".--"},   {'X', "-..-"},  {'Y', "-.--"}, {'Z', "--.."},
    {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"}, {'5', "....."},
    {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."}, {'0', "-----"},
    {'.', ".-.-.-"},{',', "--..--"},{'?', "..--.."},{'/', "-..-."}, {'-', "-....-"},
    {'=', "-...-"}, {'!', "-.-.--"},
    {0, NULL}
};

// Функция подготовки частоты CW (8-байтный пакет, совместимый с УКВ 144 МГц)
void prepare_cw_frequency(uint32_t freq_hz) {
  if (device_SI[0]) {
    if (pc_file_written || soft_restart_flag) return;
    cw_frequency_hz = freq_hz;
    uint64_t freq_mHz = (uint64_t)cw_frequency_hz * 1000ULL;
    static uint8_t cw_reg_buffer[8]; 
    calculate_freq_bytes_mHz(freq_mHz, cw_reg_buffer);
    CLK_OFF_si5351(0);               // отключить CLK0
    setFrq_si5351(cw_reg_buffer, 0); // установить частоту для CLK0
    Serial.print("[CW_ГОТОВ]: Частота несущей CW готова: "); 
    Serial.print(cw_frequency_hz); 
    Serial.println(" Hz");
  }
  else {
    //set_pio_sdr_freq(freq_hz);
    Serial.print("[CW_ГОТОВ]: Частота несущей CW готова: ");
    vfo_hardware_init(freq_hz, 10.0);
    vfo_set_cw_key(false);
    //Serial.print(fractGen_get_real_frequency()); 
    //Serial.println(" Hz");
    //fractGen_OFF();
  }
}


// Внутренняя функция задержки с опросом Serial и проверкой флагов аварии
static void cw_delay(uint32_t ms) {
    if (ms == 0) return;

    // Если длительность слишком мала (высокая скорость), используем точный микросекундный таймер
    if (ms <= 15) {
        delayMicroseconds(ms * 1000);
        return;
    }

    // Динамический контроль времени через micros() для исключения накопления дрейфа
    uint32_t start_us = micros();
    uint32_t target_us = ms * 1000;

    while (micros() - start_us < target_us) {
        if (pc_file_written || soft_restart_flag) break;
        
        // Опрашиваем Serial
        check_serial_commands();
        
        // Короткая микропауза, чтобы не перегружать ядро процессора в пустом цикле
        delayMicroseconds(500); 
    }
}


// Функция передачи одного элемента (точки или тире)
static void send_cw_element(bool is_dash) {
    if (pc_file_written || soft_restart_flag) return;


    // НАЖАТИЕ КЛЮЧА: Открываем выход генерации CLK0
    CLK_ON_si5351(0);
    digitalWrite(LED_BUILTIN, HIGH);
    ZERO_LED_RED_ON();

    // Длина тире равна 3-м точкам
    uint32_t duration = is_dash ? (CW_DOT_TIME_MS * 3) : CW_DOT_TIME_MS;
    cw_delay(duration);

    // ОТЖАТИЕ КЛЮЧА: Глушим выход генерации CLK0
    CLK_OFF_si5351(0);
    digitalWrite(LED_BUILTIN, LOW);
    ZERO_LED_OFF();

    // Обязательная пауза между элементами одного знака = 1 точка
    cw_delay(CW_DOT_TIME_MS);

}

// Посимвольный разбор и отправка строки в эфир
void send_cw_string(const char* str) {
    
    for (int i = 0; str[i] != '\0'; i++) {
        if (pc_file_written || soft_restart_flag) break;

        char c = str[i];
        if (c >= 'a' && c <= 'z') c -= 32; // Перевод в верхний регистр

        // 1. Обработка символа возврата каретки \r (просто пропускаем, чтобы не дублировать с \n)
        if (c == '\r') {
            continue; 
        }

        // 2. Если встретили перевод строки \n — принудительно подменяем его на знак равенства '='
        // изначальный знак '=' НЕ превращается в пробел.
        if (c == '\n') {
            c = '='; // Знак раздела переноса строки для Морзе (-...-)
        }

        else if (c == ' ') {
            // 3. Обработка чистого знака пробела (стандартная пауза между словами = 7 точек)
            Serial.print(" ");
            LCD_push_char(' ', 0, 16);
            cw_delay(CW_DOT_TIME_MS * 4); // 3 (из элемента) + 4 = 7 точек паузы
            continue;
        }

        // Поиск символа в таблице Морзе
        for (int j = 0; morse_table[j].c != 0; j++) {
            if (morse_table[j].c == c) {
                Serial.print(morse_table[j].c);

                LCD_push_char(morse_table[j].c, 0, 16);

                Serial.flush();

                const char* morse = morse_table[j].morse;
                for (int k = 0; morse[k] != '\0'; k++) {
                    if (pc_file_written || soft_restart_flag) break;
                    send_cw_element(morse[k] == '-');
                }
                // Стандартная пауза между буквами внутри одного слова = 3 точки
                cw_delay(CW_DOT_TIME_MS * 2); // 1 (из элемента) + 2 = 3 точки паузы
                break;
            }
        }
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("");
}

void send_cw_string(String str) {
    send_cw_string(str.c_str());
}
