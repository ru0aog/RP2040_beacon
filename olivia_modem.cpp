/**    
 * ============================================================================    
 *  olivia_modem.cpp — Модулятор Olivia 8/250 и Contestia 8/250
 *    - Логика MFSK-кодера портирована из pj_mfsk.h и pj_fht.h проекта
 *      fldigi (авторы: Pawel Jalocha SP9VRC, David Freese W1HKJ)
 *    - RSID (коды режимов 69/49) — общий rsid_send() из thor_modem.cpp, порт
 *      из rsid.cxx и rsid_defs.cxx проекта fldigi (David Freese W1HKJ)
 * ============================================================================    
 *  Общий конвейер MFSK (8 тонов, полоса 250 Гц, 31.25 бод, 32 мс/тон).    
 *  Блок = 3 символа (3 бит-плоскости). Каждый символ раскладывается на    
 *  вектор Уолша-Адамара длиной SymbolsPerBlock (SPB), скремблируется и    
 *  «размазывается» по бит-плоскостям (FreqBit+t)%3.    
 *    
 *  Различия режимов (по fldigi MFSK_Encoder / ScrambleFHT / EncodeCharacter):    
 *    Olivia 8/250    : SPB=64, Scramble=0xE257E6D0291574EC, nShift=13,    
 *                      CodeWrap=63, прямой 7-битный символ.    
 *    Contestia 8/250 : SPB=32, Scramble=0xEDB88320,         nShift=5,    
 *                      CodeWrap=31, 6-битный варикод (EncodeCharacter).    
 *    
 *  Производные работы / атрибуция:    
 *    - Логика MFSK-кодера портирована из pj_mfsk.h и pj_fht.h проекта    
 *      fldigi (авторы: Pawel Jalocha SP9VRC, David Freese W1HKJ),    
 *      распространяемых под GPLv3+.    
 *    - RSID (коды режимов 69/49) — общий rsid_send() из thor_modem.cpp, порт    
 *      из rsid.cxx и rsid_defs.cxx проекта fldigi (David Freese W1HKJ),    
 *      GPLv3+.    
 *    
 * ============================================================================    
 */
  
#include <Arduino.h>  
#include "olivia_modem.h"  
#include "thor_modem.h"  
#include "vfo_hardware.h"  
#include "si5351_driver.h"  
#include "led_blink.h"  
#include <hardware/watchdog.h>  
  
// --- Внешние зависимости проекта ---  
extern volatile bool pc_file_written;  
extern bool soft_restart_flag;  
extern void check_serial_commands();  
extern bool debug_flag;  
extern uint8_t device_SI[5];  
  
// --- Общие константы 8/250 ---  
#define OLIVIA_TONES      8  
#define OLIVIA_BITS       3        // log2(8) — бит на символ = число символов в блоке  
#define OLIVIA_SYMBOL_MS  32       // 1000 / 31.25 = 32 мс на тон  
static const double OLIVIA_STEP_HZ = 31.25; // шаг сетки = скорость = 250/8  
#define OLIVIA_PREAMBLE_BLOCKS 6   // блоков NUL-преамбулы  
// A/B-тест боковой полосы: 0 = как есть, 1 = зеркалировать номер тона  
#define OLIVIA_INVERT_TONES 0  
// Центр полосы Olivia 8/250: 3.5 тона от нижнего  
static const double OLIVIA_CENTER_OFFSET_HZ = 3.5 * OLIVIA_STEP_HZ; // 109.375

uint32_t OLIVIA_Base_freq = 3601307; // базовая частота по умолчанию  
  
// --- Переключатель режима: false = Olivia, true = Contestia ---  
static bool g_contestia = false;  
  
// Параметры текущего режима  
static inline uint16_t mode_spb()      { return g_contestia ? 32u : 64u; }  
static inline uint64_t mode_scramble() { return g_contestia ? 0xEDB88320ULL  
                                                            : 0xE257E6D0291574ECULL; }  
static inline uint8_t  mode_nshift()   { return g_contestia ? 5u : 13u; }  



// --- RSID (Reed-Solomon ID) ---  
// Общий генератор rsid_send() из thor_modem.cpp (порт rsid.cxx fldigi).
// Коды мод из rsid_defs.cxx:  
#define RSID_CODE_OLIVIA_8_250    69  
#define RSID_CODE_CONTESTIA_8_250 49  

// RSID для текущего режима (g_contestia)
// центр полосы по конвенции fldigi = нижний тон + N/2·шаг = base + 125 Гц
static bool olivia_rsid() {  
    if (device_SI[0]) return true;          // Si5351-ветка: Olivia работает только через PIO
    uint16_t code = g_contestia ? RSID_CODE_CONTESTIA_8_250  
                                : RSID_CODE_OLIVIA_8_250;  

    // Base_freq — уже центр полосы: сетка строится как base ± 3.5·step    
    if (!rsid_send(code, (double)OLIVIA_Base_freq)) return false;

    // вернуть сетку данных 8/31.25 (нижний тон = центр - 109.375 Гц)
    vfo_hardware_init(OLIVIA_Base_freq - OLIVIA_CENTER_OFFSET_HZ, OLIVIA_STEP_HZ);
    vfo_set_tone_instant(0);  
    return true;  
}

// --- Матрицы Уолша-Адамара: 64x64 (Olivia) и 32x32 (Contestia) ---  
static uint64_t walsh64[64];  
static uint64_t walsh32[32];  
static bool     walsh_ready = false;  
  
static void build_walsh(uint64_t* tbl, int n) {  
    // IFHT(δ_i)[j] = (-1)^popcount(i) * (-1)^popcount(i & j)  
    // => бит выхода (sample<0) = parity(i) XOR parity(i & j)  
    for (int i = 0; i < n; i++) {  
        uint64_t row = 0;  
        uint8_t pi = __builtin_parity((unsigned)i);  
        for (int j = 0; j < n; j++) {  
            if (__builtin_parity((unsigned)(i & j)) ^ pi)  
                row |= (1ULL << (n - 1 - j));  
        }  
        tbl[i] = row;  
    }  
}  
  
static void olivia_build_walsh() {  
    build_walsh(walsh64, 64);  
    build_walsh(walsh32, 32);  
    walsh_ready = true;  
}  
  
// --- Грей-код ---  
static inline uint8_t to_gray(uint8_t v) { return v ^ (v >> 1); }  
  
// --- Варикод Contestia: символ -> 0..63 (из fldigi EncodeCharacter) ---  
static uint8_t contestia_encode(char ch) {  
    uint8_t c = (uint8_t)ch;  
    if (c >= 'a' && c <= 'z') c += 'A' - 'a';  
    if (c == ' ')               return 59;  
    else if (c == '\r')         return 60;  
    else if (c == '\n')         return 0;  
    else if (c >= 33 && c <= 90) return c - 32;  
    else if (c == 8)            return 61;  
    else if (c == 0)            return 0;  
    else                        return (uint8_t)('?' - 32); // = 31  
}  
  
// --- Накопитель блока: 3 символа ---  
static char    block_chars[OLIVIA_BITS];  
static uint8_t block_count = 0;  
  
// Общая подготовка сетки из 8 тонов (PIO-VFO или Si5351)  
static void prepare_common(uint32_t base_hz, const char* tag) {  
    OLIVIA_Base_freq = base_hz;
    if (!walsh_ready) olivia_build_walsh();  
    block_count = 0;  
  
    if (!device_SI[0]) {  
        vfo_hardware_init(OLIVIA_Base_freq - OLIVIA_CENTER_OFFSET_HZ, OLIVIA_STEP_HZ);  
        vfo_set_tone_instant(0);  
        if (debug_flag) {  
            Serial.print('['); Serial.print(tag);  
            Serial.print(F("] PIO готов, центр "));  
            Serial.print(OLIVIA_Base_freq); Serial.print(F(" Гц, шаг "));  
            Serial.print(OLIVIA_STEP_HZ, 3); Serial.println(F(" Гц"));  
        }  
    }  
}  
  
void prepare_olivia_frequencies(uint32_t base_hz) {  
    g_contestia = false;  
    prepare_common(base_hz, "OLIVIA");  
}  
  
void prepare_contestia_frequencies(uint32_t base_hz) {  
    g_contestia = true;  
    prepare_common(base_hz, "CONTESTIA");  
}  
  
// Абсолютный дедлайн следующего символа (мкс)  
static uint32_t olivia_next_deadline_us = 0;  
  
// Удержание одного тона 32 мс с кормлением собаки и опросом CLI  
static inline bool olivia_emit_tone(uint8_t tone) {  
    vfo_set_tone_instant(tone);  
   
    if (g_contestia) {ZERO_LED_CYAN_ON();} else {ZERO_LED_YELLOW_ON();}

    uint32_t led_off_us = olivia_next_deadline_us - (uint32_t)OLIVIA_SYMBOL_MS * 500UL;  
    bool led_half_off = false;  
    while ((int32_t)(micros() - olivia_next_deadline_us) < 0) {  
        if (pc_file_written || soft_restart_flag) { ZERO_LED_OFF(); return false; }  
        if (Serial.available() > 0) { ZERO_LED_OFF(); soft_restart_flag = true; return false; }  
        if (!led_half_off && (int32_t)(micros() - led_off_us) >= 0) {  
            ZERO_LED_OFF();  
            led_half_off = true;  
        }  
        delayMicroseconds(20);  
        watchdog_update();  
    }  
    if (!led_half_off) ZERO_LED_OFF();  
    olivia_next_deadline_us += (uint32_t)OLIVIA_SYMBOL_MS * 1000UL;  
    return true;  
}  
  
static bool olivia_flush_block() {
    if (block_count == 0) return true;  
  
    // fldigi добивает блок НУЛЁМ (idle)  
    for (uint8_t p = block_count; p < OLIVIA_BITS; p++) block_chars[p] = 0;  
  
    const uint16_t SPB      = mode_spb();          // 64 (Olivia) / 32 (Contestia)  
    const uint16_t WRAP     = SPB - 1;             // CodeWrap  
    const uint64_t SCRAMBLE = mode_scramble();  
    const uint8_t  NSHIFT   = mode_nshift();  
    const uint64_t* WALSH   = g_contestia ? walsh32 : walsh64;  
  
    static uint8_t out[64];  
    for (int t = 0; t < SPB; t++) out[t] = 0;  
  
    for (uint8_t freqBit = 0; freqBit < OLIVIA_BITS; freqBit++) {  
        uint16_t v;  
        if (g_contestia) v = contestia_encode(block_chars[freqBit]);        // 0..63  
        else             v = (uint8_t)block_chars[freqBit] & 0x7Fu;         // 0..127  
  
        uint16_t idx = v & WRAP;            // строка Адамара  
        uint8_t  inv = (v >= SPB) ? 1 : 0;  // 1 => отрицательный импульс  
        
        uint64_t row = WALSH[idx];  
        if (inv) {
            if (g_contestia) {
                // Для Контестии инвертируем только ее младшие 32 бита, 
                // оставляя старшую половину строго в нулях!
                row = (~row) & 0x00000000FFFFFFFFULL; 
            } else {
                // Для Оливии инвертируем всю 64-битную строку, как в вашем первом коде
                row = ~row; 
            }
        }
  
        // ScrambleFHT: codeBit = (freqBit*nShift + t) & CodeWrap  
        uint8_t codeBit = (uint8_t)((freqBit * NSHIFT) & WRAP);  
        for (int t = 0; t < SPB; t++) {  
            uint8_t neg = (uint8_t)((row >> (WRAP - t)) & 1ULL);   // знак отсчёта FHT  
            
            neg ^= (uint8_t)((SCRAMBLE >> codeBit) & 1ULL);        // ^ ScramblingCode  
            if (neg) {                                             // FHT_Buffer[t] < 0  
                uint8_t bit = freqBit + (uint8_t)(t % OLIVIA_BITS); // (FreqBit + Rotate)  
                if (bit >= OLIVIA_BITS) bit -= OLIVIA_BITS;  
                out[t] |= (uint8_t)(1u << bit);  
            }  
            codeBit = (uint8_t)((codeBit + 1) & WRAP);  
        }  
    }  
  
    // Выдаём SPB тонов; модулятор fldigi применяет Грей-код к номеру тона  
    for (int t = 0; t < SPB; t++) {  
        uint8_t tone = to_gray(out[t]) & (OLIVIA_TONES - 1);  
#if OLIVIA_INVERT_TONES  
        tone = (uint8_t)((OLIVIA_TONES - 1) - tone);  
#endif  
        if (!olivia_emit_tone(tone)) { block_count = 0; return false; }  
    }  
    block_count = 0;  
    return true;  
}
  
void olivia_send_char(char c) {  
    if (pc_file_written || soft_restart_flag) return;  
    if (c != '\r' && c != '\n' && c != 0) Serial.print(c);  
    block_chars[block_count++] = c;  
    if (block_count >= OLIVIA_BITS) olivia_flush_block();  
}  
  
// Общий передатчик строки (режим задаётся g_contestia через prepare_*)  
static void send_string_common(const char* str) {  
    if (str == nullptr) return;  
    if (!walsh_ready) olivia_build_walsh();  
    block_count = 0;  
  
    vfo_set_narrowband(true);  
  
    if (!device_SI[0]) {  
        vfo_operation_set(false);  
        vfo_set_tone_instant(0);  
    } else {  
        VFO_TX_ON();  
    }  
  
    // RSID-маркер режима ДО преамбулы: fldigi сам переключит  
    // Olivia<->Contestia и отцентрует приём (RxID)  
    if (!olivia_rsid()) goto tx_done;  
  
    olivia_next_deadline_us = micros() + (uint32_t)OLIVIA_SYMBOL_MS * 1000UL;  
  
    // Преамбула — idle-символы NUL (0x00)  
    for (int i = 0; i < OLIVIA_BITS * OLIVIA_PREAMBLE_BLOCKS; i++)  
        olivia_send_char((char)0); 
  
    // Полезная нагрузка  
    while (*str && !pc_file_written && !soft_restart_flag) {  
        olivia_send_char(*str++);  
    }  
  
    if (block_count > 0) olivia_flush_block();  
  
    // Постамбула — 2 блока NUL  
    for (int i = 0; i < OLIVIA_BITS * 2 && !pc_file_written && !soft_restart_flag; i++)  
        olivia_send_char((char)0);  
    if (block_count > 0) olivia_flush_block();  

    tx_done:  
    VFO_TX_OFF();               // глушим генераторы
    vfo_set_narrowband(false);  // возвращаем дизер
    ZERO_LED_OFF();
    Serial.println("");  
}  
  
void olivia_send_string(const char* str) {  
    g_contestia = false;  
    send_string_common(str);  
}  
void olivia_send_string(String str) { olivia_send_string(str.c_str()); }  
  
void contestia_send_string(const char* str) {  
    g_contestia = true;  
    send_string_common(str);  
}  
void contestia_send_string(String str) { contestia_send_string(str.c_str()); }