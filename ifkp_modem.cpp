#include "ifkp_modem.h"
#include "si5351_driver.h"
#include "file_manager.h"

uint8_t current_tone = 0;
uint32_t IFKP_Base_freq = 3601307;
uint8_t DATA_IFKP[33][8];

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
    CLK_ON_si5351(0);       // разрешить выход частоты на CLK0
}

// Передача разницы (дельта) частоты с микрозадержками и мгновенным выходом
void send_delta(uint8_t delta) {
    // Прерываем работу, если зафиксирована активность ПК или подан сигнал рестарта
    if (pc_file_written || soft_restart_flag) return; 

    if (SI_FAIL == false) {
        current_tone = (current_tone + delta) % 33;
        set_ifkp_tone(current_tone);
        
        // 1. Включаем светодиод и ждем 250 мс
        digitalWrite(LED_BUILTIN, HIGH);
        for (int i = 0; i < 50; i++) {
            if (pc_file_written || soft_restart_flag) break;
            check_serial_commands(); 
            delay(5);
        }
        
        // 2. Выключаем светодиод и ждем еще 250 мс
        digitalWrite(LED_BUILTIN, LOW);
        for (int i = 0; i < 50; i++) {
            if (pc_file_written || soft_restart_flag) break;
            check_serial_commands();
            delay(5);
        }
    }
}

// Заполнение таблицы базовых частот DATA_IFKP (совместимо с УКВ 144 МГц)
void prepare_ifkp_frequencies(uint32_t base_hz) {
    uint64_t base_mHz = (uint64_t)base_hz * 1000ULL;
    Serial.println(F("[IFKP_СЕТКА] Расчет и вывод всех 33 тонов IFKP:"));
    for (int i = 0; i < 33; i++) {
        // Шаг 386/33 = 11.697 Гц
        uint64_t freq_mHz = base_mHz + ((uint64_t)i * 11697ULL);
        calculate_freq_bytes_mHz(freq_mHz, DATA_IFKP[i]);
        // Математически точный перевод миллигерц обратно в Гц для вывода (без double)
        uint32_t hz_part = (uint32_t)(freq_mHz / 1000ULL);
        uint32_t mhz_part = (uint32_t)(freq_mHz % 1000ULL);
        // Пошагово выводим каждый тон от F_00 до F_32
        char tone_buf[64];
        snprintf(tone_buf, sizeof(tone_buf), "            F_%02d: %lu.%03lu Hz", i, (unsigned long)hz_part, (unsigned long)mhz_part);
        Serial.println(tone_buf);
    }
    Serial.println(F("[IFKP_ГОТОВ] 33 тона сетки частот IFKP успешно рассчитаны."));
}

// Модулятор протокола IFKP
void send_ifkp_char(char c) {
  if (SI_FAIL == false) {
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
}

// Посимвольный перебор строки
void send_ifkp_string(const char* str) {
  if (SI_FAIL == false) {
    for (int i = 0; str[i] != '\0'; i++) {
        if (pc_file_written || soft_restart_flag) {
          break; // Выходим через break, чтобы гарантированно проскочить вниз к отключению Si5351
        }
        send_ifkp_char(str[i]);
    }

    // Глушим генерацию Si5351 (Регистр 0x03 = 0xFF).
    si5351_write_reg(0x03, 0xFF);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(""); 
  }
}

// Функция-помощник для объектов String
void send_ifkp_string(String str) {
  send_ifkp_string(str.c_str());
}
