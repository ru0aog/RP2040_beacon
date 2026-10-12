/**  
 * ============================================================================  
 *  olivia_modem.cpp — Модулятор Olivia 8/250 и Contestia 8/250  
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
 * ============================================================================  
 */  
  
#include <Arduino.h>  
#include "olivia_modem.h"  
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
#define OLIVIA_PREAMBLE_BLOCKS 10   // блоков NUL-преамбулы  
// A/B-тест боковой полосы: 0 = как есть, 1 = зеркалировать номер тона  
#define OLIVIA_INVERT_TONES 0  
  
uint32_t OLIVIA_Base_freq = 3601307; // базовая частота по умолчанию  
  
// --- Переключатель режима: false = Olivia, true = Contestia ---  
static bool g_contestia = false;  
  
// Параметры текущего режима  
static inline uint16_t mode_spb()      { return g_contestia ? 32u : 64u; }  
static inline uint64_t mode_scramble() { return g_contestia ? 0xEDB88320ULL  
                                                            : 0xE257E6D0291574ECULL; }  
static inline uint8_t  mode_nshift()   { return g_contestia ? 5u : 13u; }  
  
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
        vfo_hardware_init(base_hz, OLIVIA_STEP_HZ);  
        vfo_set_tone_instant(0);  
        if (debug_flag) {  
            Serial.print('['); Serial.print(tag);  
            Serial.print(F("] PIO готов, база "));  
            Serial.print(base_hz); Serial.print(F(" Гц, шаг "));  
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
    ZERO_LED_GREEN_ON();  
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
  
// Чистое кодирование блока из 3 символов -> SPB «сырых» символов (0..7, ДО Грея).  
// chars[0..2] — входные символы; out[0..SPB-1] — результат; contestia — режим.  
// Возвращает SPB. НИЧЕГО не шлёт в эфир (используется и эфиром, и самотестом).  
static uint16_t olivia_encode_block(const char* chars, uint8_t* out, bool contestia) {  
    const uint16_t SPB      = contestia ? 32u : 64u;  
    const uint16_t WRAP     = SPB - 1;  
    const uint64_t SCRAMBLE = contestia ? 0xEDB88320ULL : 0xE257E6D0291574ECULL;  
    const uint8_t  NSHIFT   = contestia ? 5u : 13u;  
    const uint64_t* WALSH   = contestia ? walsh32 : walsh64;  
  
    for (int t = 0; t < SPB; t++) out[t] = 0;  
  
    for (uint8_t freqBit = 0; freqBit < OLIVIA_BITS; freqBit++) {  
        uint16_t v;  
        if (contestia) v = contestia_encode(chars[freqBit]);  
        else           v = (uint8_t)chars[freqBit] & 0x7Fu;  
  
        uint16_t idx = v & WRAP;  
        uint8_t  inv = (v >= SPB) ? 1 : 0;  
        uint64_t row = inv ? ~WALSH[idx] : WALSH[idx];  
  
        uint8_t codeBit = (uint8_t)((freqBit * NSHIFT) & WRAP);  
        for (int t = 0; t < SPB; t++) {  
            uint8_t neg = (uint8_t)((row >> (WRAP - t)) & 1ULL);  
            neg ^= (uint8_t)((SCRAMBLE >> codeBit) & 1ULL);  
            if (neg) {  
                uint8_t bit = freqBit + (uint8_t)(t % OLIVIA_BITS);  
                if (bit >= OLIVIA_BITS) bit -= OLIVIA_BITS;  
                out[t] |= (uint8_t)(1u << bit);  
            }  
            codeBit = (uint8_t)((codeBit + 1) & WRAP);  
        }  
    }  
    return SPB;  
}

// Прямой FHT (как в fldigi pj_fht.h) — для раскодирования в самотесте  
static void fht_forward(int16_t* d, int n) {  
    for (int step = 1; step < n; step *= 2)  
        for (int p = 0; p < n; p += 2 * step)  
            for (int q = p; (q - p) < step; q++) {  
                int16_t b1 = d[q], b2 = d[q + step];  
                d[q] = (int16_t)(b2 + b1);  
                d[q + step] = (int16_t)(b2 - b1);  
            }  
}  
  
// Раскодировать freqBit-ю бит-плоскость из массива символов out[] (зеркало DecodeCharacter).  
// Возвращает исходное значение v (0..2*SPB-1): idx = позиция пика, +SPB если пик отрицательный.  
static uint16_t olivia_decode_freqbit(const uint8_t* out, uint8_t freqBit, bool contestia) {  
    const uint16_t SPB      = contestia ? 32u : 64u;  
    const uint16_t WRAP     = SPB - 1;  
    const uint64_t SCRAMBLE = contestia ? 0xEDB88320ULL : 0xE257E6D0291574ECULL;  
    const uint8_t  NSHIFT   = contestia ? 5u : 13u;  
  
    static int16_t fht[64];  
    uint8_t rotate  = freqBit;  
    uint8_t codeBit = (uint8_t)((freqBit * NSHIFT) & WRAP);  
    for (int t = 0; t < SPB; t++) {  
        uint8_t b = (uint8_t)((out[t] >> rotate) & 1u); // 1 => отсчёт FHT был < 0  
        int16_t val = b ? -1 : +1;  
        if ((SCRAMBLE >> codeBit) & 1ULL) val = (int16_t)(-val); // снять скремблер  
        fht[t] = val;  
        codeBit = (uint8_t)((codeBit + 1) & WRAP);  
        rotate++; if (rotate >= OLIVIA_BITS) rotate -= OLIVIA_BITS;  
    }  
    fht_forward(fht, SPB);  
  
    int16_t peak = 0; uint16_t pidx = 0;  
    for (int i = 0; i < SPB; i++) {  
        int16_t a = fht[i] < 0 ? (int16_t)-fht[i] : fht[i];  
        int16_t p = peak   < 0 ? (int16_t)-peak   : peak;  
        if (a > p) { peak = fht[i]; pidx = (uint16_t)i; }  
    }  
    return (peak < 0) ? (uint16_t)(pidx + SPB) : pidx;  
}  
  
// Round-trip самотест: кодируем блок -> раскодируем -> сверяем. Печать в Serial.  
void olivia_selftest(bool contestia, const char* s) {  
    if (!walsh_ready) olivia_build_walsh();  
    const uint16_t SPB = contestia ? 32u : 64u;  
    static uint8_t out[64];  
  
    Serial.print(F("[SELFTEST] "));  
    Serial.print(contestia ? F("CONTESTIA SPB=32  \"") : F("OLIVIA SPB=64  \""));  
    Serial.print(s); Serial.println(F("\""));  
  
    int bad = 0;  
    for (const char* p = s; ; ) {  
        char blk[OLIVIA_BITS];  
        int n = 0;  
        for (; n < OLIVIA_BITS && *p; n++) blk[n] = *p++;  
        if (n == 0) break;  
        for (int k = n; k < OLIVIA_BITS; k++) blk[k] = 0; // добивка idle  
  
        olivia_encode_block(blk, out, contestia);  
  
        for (uint8_t fb = 0; fb < OLIVIA_BITS; fb++) {  
            uint16_t vin  = contestia ? contestia_encode(blk[fb])  
                                      : ((uint8_t)blk[fb] & 0x7Fu);  
            uint16_t vout = olivia_decode_freqbit(out, fb, contestia);  
            bool ok = (vin == vout);  
            if (!ok) bad++;  
            Serial.print(F("  ch='"));  
            char c = blk[fb];  
            Serial.print((c >= 32 && c < 127) ? c : '.');  
            Serial.print(F("' freqBit=")); Serial.print(fb);  
            Serial.print(F(" vin="));  Serial.print(vin);  
            Serial.print(F(" vout=")); Serial.print(vout);  
            Serial.println(ok ? F("  OK") : F("  <<< MISMATCH"));  
        }  
        if (!*p) break;  
    }  
    Serial.print(F("[SELFTEST] несовпадений: ")); Serial.println(bad);  
    Serial.println(bad == 0 ? F("[SELFTEST] КОДЕР ВЕРЕН -> баг в RF/боковой/преамбуле")  
                            : F("[SELFTEST] КОДЕР РАСХОДИТСЯ -> баг в битах кодера"));  
}

  
// Передача одного блока из 3 символов (SPB тонов)  
static bool olivia_flush_block() {  
    if (block_count == 0) return true;  
  
    // fldigi добивает блок НУЛЁМ (idle)  
    for (uint8_t p = block_count; p < OLIVIA_BITS; p++) block_chars[p] = 0;  
  
    static uint8_t out[64];  
    uint16_t SPB = olivia_encode_block(block_chars, out, g_contestia);  
  
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
        vfo_operation_set(true);  
    } else {  
        VFO_TX_ON();  
    }  
  
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
  
    if (device_SI[0]) VFO_TX_OFF();  
    else              vfo_operation_set(false);  
    vfo_set_narrowband(false);  
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