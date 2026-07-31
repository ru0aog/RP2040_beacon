#include "cw_modem.h"
#include "si5351_driver.h"
#include "file_manager.h"

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


// Внутренняя функция задержки с опросом Serial и проверкой флагов аварии
static void cw_delay(uint32_t ms) {
    // Если длительность слишком мала (высокая скорость), используем точный микросекундный таймер
    if (ms <= 15) {
        delayMicroseconds(ms * 1000);
        return;
    }

    // Для обычных пауз проверяем флаги и Serial каждые 2 миллисекунды (вместо 5)
    uint32_t chunks = ms / 2;
    uint32_t remainder = ms % 2;

    for (uint32_t i = 0; i < chunks; i++) {
        if (pc_file_written || soft_restart_flag) break;
        check_serial_commands();
        delay(2);
    }
    if (remainder > 0 && !pc_file_written && !soft_restart_flag) {
        delay(remainder);
    }
}


// Функция передачи одного элемента (точки или тире)
static void send_cw_element(bool is_dash) {
    if (pc_file_written || soft_restart_flag) return;

    if (SI_FAIL == false) {
        // НАЖАТИЕ КЛЮЧА: Открываем выход генерации CLK0
        CLK_ON_si5351(0);
        digitalWrite(LED_BUILTIN, HIGH);

        // Длина тире равна 3-м точкам
        uint32_t duration = is_dash ? (CW_DOT_TIME_MS * 3) : CW_DOT_TIME_MS;
        cw_delay(duration);

        // ОТЖАТИЕ КЛЮЧА: Глушим выход генерации CLK0
        CLK_OFF_si5351(0);
        digitalWrite(LED_BUILTIN, LOW);

        // Обязательная пауза между элементами одного знака = 1 точка
        cw_delay(CW_DOT_TIME_MS);
    }
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
        if (c == '\n') {
            c = '='; 
        }

        // 2. Если встретили = — принудительно меняем на пробел
        if (c == '=') {
            c = ' '; 
        }

        // 3. Обработка чистого знака пробела (стандартная пауза между словами = 7 точек)
        if (c == ' ') {
            Serial.print(" ");
            cw_delay(CW_DOT_TIME_MS * 4); // 3 (из элемента) + 4 = 7 точек паузы
            continue;
        }

        // Поиск символа в таблице Морзе
        for (int j = 0; morse_table[j].c != 0; j++) {
            if (morse_table[j].c == c) {
                Serial.print(morse_table[j].c);
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
