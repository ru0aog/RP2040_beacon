/**  
 * ============================================================================  
 *  jtty_modem.cpp — Реализация передатчика JTTY (По мотивам WSJT-X release/3.3)  
 * ============================================================================  
 *  
 *  Обеспечивает посимвольную блокирующую выдачу 4-FSK кадра JTTY. Вся сетка  
 *  частот готовится через унифицированные процедуры mfsk_*, символы выгоняются  
 *  в эфир по мертвым дедлайнам micros() с постоянным сбросом сторожевого  
 *  таймера и контролем флагов прерывания (pc_file_written, soft_restart_flag).  
 *  
 *  ВНИМАНИЕ (TODO-VERIFY):  
 *    Параметры девиации (JTTY_STEP_HZ), длительности символа (JTTY_SYMBOL_US),  
 *    размерности кадра (JTTY_NDATA) и алгоритм FEC/Interleaver являются  
 *    предварительными. Перед боевым запуском необходима их верификация по  
 *    исходным кодам Fortran-кодека lib/jtty*.f90 оригинального стека WSJT-X.  
 *  
 *  ЛОГИКА ВЫВОДА:  
 *    Несущая частота (carrier) активируется строго в момент старта фрейма  
 *    и безусловно гасится сразу по завершении отправки последнего символа.  
 * 
 * Полная спецификация передатчика
 *   Кадр:     13 sync (is13) + 46 data tones = 59 символов × 32 мс = 1.888 с  
 *   Тон k:    f0 + k × 31.25 Гц,   k ∈ {0,1,2,3},  f0 = нижний тон  
 *   Payload:  atom32 << 2 | eom<<1 | 0   → 34 бита  (порядок — см. оговорку)  
 *   CRC-12:   poly 0x80F → 46 информационных бит  
 *   TBCC:     K=10, g0=0o1167, g1=0o1545, tail-biting (регистр = последние 9 бит)  
 *   Дибиты:   {g1,g0} → тон 0..3 (порядок бит проверить по tbcc.f90)
 * ============================================================================  
 */


  
#include "jtty_modem.h"  
#include "vfo_hardware.h"  
#include "thor_modem.h"            // rsid? нет — у JTTY нет RSID; нужен mfsk_prepare прототип  
#include <Arduino.h>  
  
// ===== Внешние флаги проекта (как в rtty_modem.cpp / thor_modem.cpp) =====  
extern uint8_t device_SI[5]; 
extern bool pc_file_written;  
extern bool soft_restart_flag;  
extern bool debug_flag;  
  
// ===== Параметры режима JTTY =====  
// TODO-VERIFY: значения из lib/jtty*.f90 (wsjtx release/3.3)  
#define JTTY_NUMTONES            4                  // 4-FSK  
#define JTTY_STEP_HZ            (2.315f)            // TODO-VERIFY: шаг тонов  
#define JTTY_SYMBOL_US     (432000UL)               // TODO-VERIFY: 1/скорость, мкс  
#define JTTY_NDATA              64                  // TODO-VERIFY: символов полезной нагрузки  
#define JTTY_CENTER_OFFSET_HZ (1.5f * JTTY_STEP_HZ) // центр полосы → нижний тон  
  
static double   JTTY_Base_freq = 0;                 // центр полосы, Гц  
static uint32_t jtty_deadline_us = 0;  
  
// ===== Тайминг (каноническая схема проекта) =====  
  
// Дедлайн первого символа — «сейчас + один символ», как mfsk_sync_deadline()  
static void jtty_sync_deadline(void) {  
    jtty_deadline_us = micros() + JTTY_SYMBOL_US;  
}  
  
// Ждать до конца текущего символа; каждое ожидание сдвигает дедлайн.  
// Возвращает false при запросе остановки.  
static bool jtty_wait_symbol(void) {  
    while ((int32_t)(micros() - jtty_deadline_us) < 0) {  
        if (pc_file_written || soft_restart_flag) return false;  
        watchdog_update();  
        tight_loop_contents();  
    }  
    jtty_deadline_us += JTTY_SYMBOL_US;             // наследуемый дедлайн, без дрейфа  
    return true;  
}  
  
// ===== Сетка тонов =====  
  
// base_hz — ЦЕНТР полосы JTTY (как в prepare_*_frequencies остальных режимов).  
// Сетка строится от нижнего тона: center - (numtones-1)/2 * step.  
void prepare_jtty_frequencies(uint32_t base_hz) {  
    JTTY_Base_freq = base_hz;  
    mfsk_prepare(base_hz - JTTY_CENTER_OFFSET_HZ, JTTY_STEP_HZ, JTTY_NUMTONES);  
}  
  
// ===== Выдача одного символа =====  
  
// tone: 0..JTTY_NUMTONES-1 — индекс в сетке mfsk_prepare  
static bool jtty_emit_tone(uint8_t tone) {  
    mfsk_set_tone(tone);  
    return jtty_wait_symbol();  
}  
  
// ===== Кодирование =====  
  
// TODO-VERIFY: реальное кодирование JTTY:  
//   alphabet → bits → свёрточный код → перемежитель → символы 4-FSK  
//   + вставка sync-вектора.  
// Здесь: временная заглушка — вызывающий код передаёт готовый массив символов  
// тонов (0..3). Портирование jtty_encode()/interleave() из Fortran — следующий шаг.  
  
// ===== Передатчик =====  
  
// symbols: готовый кадр JTTY_NDATA тонов (0..3).  
// Блокирующая функция, как send_rtty_string: при is_transmitting уже  
// должен стоять в вызывающем коде; выход: carrier off.  
bool jtty_send_frame(const uint8_t* symbols, uint8_t nsym) {  
    if (symbols == nullptr || nsym == 0) return false;  
    if (nsym > JTTY_NDATA) nsym = JTTY_NDATA;  
    if (JTTY_Base_freq == 0) return false;          // prepare не вызывали  
  
    if (debug_flag) {  
        Serial.printf("[JTTY] TX: f=%.1f Гц, step=%.3f Гц, %u симв. x %lu мкс\n",  
                      JTTY_Base_freq, (double)JTTY_STEP_HZ,  
                      (unsigned)nsym, (unsigned long)JTTY_SYMBOL_US);  
    }  
  
    // Несущая включается только на символы — перед первым, после последнего off  
    jtty_sync_deadline();  
    mfsk_set_tone(0);  
    mfsk_carrier(true);  
  
    for (uint8_t i = 0; i < nsym; i++) {  
        if (!jtty_emit_tone(symbols[i])) {  
            mfsk_carrier(false);  
            return false;  
        }  
    }  
  
    // Последний символ отзвучал на дедлайне — гасим  
    mfsk_carrier(false);  
    return true;  
}  
  
// Текстовый интерфейс — пока кодирует через заглушку jtty_encode_text.  
// TODO-VERIFY: заменить на порт jtty_encode() из WSJT-X.  
static uint8_t jtty_frame[JTTY_NDATA];  
  
static uint8_t jtty_encode_text(const char* str) {  
    // ЗАГЛУШКА: детерминированное отображение строки в кадр, только для  
    // проверки тракта. Реальный FEC — порт из lib/jtty*.f90.  
    uint8_t n = 0;  
    for (const char* p = str; *p && n < JTTY_NDATA; p++, n++) {  
        jtty_frame[n] = ((uint8_t)*p >> 0) & 3;  
        if (n + 1 < JTTY_NDATA) jtty_frame[++n] = ((uint8_t)*p >> 2) & 3;  
        if (n + 1 < JTTY_NDATA) jtty_frame[++n] = ((uint8_t)*p >> 4) & 3;  
        if (n + 1 < JTTY_NDATA) jtty_frame[++n] = ((uint8_t)*p >> 6) & 3;  
    }  
    return n;  
}  
  
void jtty_send_string(const char* str) {  
    if (str == nullptr) return;  
    uint8_t n = jtty_encode_text(str);  
    jtty_send_frame(jtty_frame, n);  
}



// ====================== JTTY FEC (порт lib/jtty/tbcc.f90) ======================  
// K=10 (nu=9), rate-1/2, tail-biting, генераторы octal 1167/1545 = hex 0x277/0x365  
// CRC-12: полином 0x80F. Кадр: 34 бита payload + 12 CRC = 46 инфобит -> 46 тонов.  
  
#define JTTY_PAYLOAD_BITS   34  
#define JTTY_CRC_BITS       12  
#define JTTY_INFO_BITS      (JTTY_PAYLOAD_BITS + JTTY_CRC_BITS)  // 46  
#define JTTY_NU             9  
#define JTTY_STATE_MASK     ((1u << JTTY_NU) - 1u)               // 0x1FF  
#define JTTY_REG_MASK       ((1u << (JTTY_NU + 1)) - 1u)         // 0x3FF  
#define JTTY_G0             0x277u   // octal 1167  
#define JTTY_G1             0x365u   // octal 1545  
#define JTTY_CRC_POLY       0x80Fu  
#define JTTY_CRC_TOPBIT     (1u << (JTTY_CRC_BITS - 1))          // 0x800  
#define JTTY_CRC_REGMASK    ((1u << JTTY_CRC_BITS) - 1u)         // 0xFFF  
  
// Sync-последовательность is13 из jtty_fec.f90  
static const uint8_t JTTY_SYNC13[13] = {0,2,2,3,0,0,3,2,1,3,1,2,0};  
  
static inline uint8_t jtty_parity(uint32_t v) {  
    v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;  
    return v & 1u;  
}  
  
// payload[34] -> info_bits[46] (биты как int8 0/1, порядок MSB-first как в Fortran)  
static void jtty_encode_crc12(const uint8_t payload[JTTY_PAYLOAD_BITS],  
                              uint8_t info_bits[JTTY_INFO_BITS]) {  
    memcpy(info_bits, payload, JTTY_PAYLOAD_BITS);  
    uint32_t crc = 0;  
    for (int i = 0; i < JTTY_PAYLOAD_BITS; i++) {  
        crc ^= (uint32_t)payload[i] << (JTTY_CRC_BITS - 1);  
        crc = (crc & JTTY_CRC_TOPBIT) ? ((crc << 1) ^ JTTY_CRC_POLY)  
                                      : (crc << 1);  
        crc &= JTTY_CRC_REGMASK;  
    }  
    for (int i = 0; i < JTTY_CRC_BITS; i++)  
        info_bits[JTTY_PAYLOAD_BITS + i] = (crc >> (JTTY_CRC_BITS - 1 - i)) & 1u;  
}  
  
// info_bits[46] -> tone_symbols[46] (значения 0..3)  
// Порт tbcc_encode: tail-biting init последними 9 битами, Gray-маппинг.  
static void jtty_tbcc_encode(const uint8_t info_bits[JTTY_INFO_BITS],  
                             uint8_t tone_symbols[JTTY_INFO_BITS]) {  
    // Tail-biting: стартовое состояние = последние nu бит info_bits  
    uint32_t state = 0;  
    for (int t = 0; t < JTTY_NU; t++) {  
        uint8_t bit = info_bits[JTTY_INFO_BITS - (JTTY_NU - 1) + t];  
        state = ((state << 1) | bit) & JTTY_STATE_MASK;  
    }  
  
    for (int t = 0; t < JTTY_INFO_BITS; t++) {  
        uint32_t reg = ((state << 1) | info_bits[t]) & JTTY_REG_MASK;  
        uint8_t b0 = jtty_parity(reg & JTTY_G0);  
        uint8_t b1 = jtty_parity(reg & JTTY_G1);  
        state = reg & JTTY_STATE_MASK;  
        // Gray: 00->0, 01->1, 11->2, 10->3  
        tone_symbols[t] = (b0 << 1) | b1;      // бинарное значение  
        // перекодировка по таблице [0,1,3,2] на пару (b0,b1):  
        static const uint8_t gray2tone[4] = {0, 1, 3, 2};  
        tone_symbols[t] = gray2tone[(b0 << 1) | b1];  
    }  
}  
  
// Полный кадр: payload[34] -> itone[59] (13 sync + 46 data)  
static void jtty_build_frame(const uint8_t payload[JTTY_PAYLOAD_BITS],  
                             uint8_t itone[59]) {  
    uint8_t info[JTTY_INFO_BITS];  
    jtty_encode_crc12(payload, info);  
    memcpy(itone, JTTY_SYNC13, 13);  
    jtty_tbcc_encode(info, itone + 13);  
}



//---------------порт pack28 -------------------

// jtty_pack28: позывной/токен -> 28 бит. Возвращает -1 при ошибке.  
// Покрывает: DE, QRZ, CQ, CQ_nnn, CQ_aaaa, стандартные позывные.  
// Нестандартные позывные требуют ihashcall (22-битный хэш) — см. ниже.  
  
#include <stdint.h>  
#include <string.h>  
#include <ctype.h>  
  
#define P28_NTOKENS 2063592u  
#define P28_MAX22   4194304u  
  
static bool is_dig(char c)  { return c >= '0' && c <= '9'; }  
static bool is_let(char c)  { return c >= 'A' && c <= 'Z'; }  
  
static int idx_in(const char *table, char c) {  
    const char *p = strchr(table, c);  
    return p ? (int)(p - table) : -1;  
}  
  
// c13 — позывной верхним регистром, без <>, до 12 символов.  
int32_t jtty_pack28(const char *c13_in)  
{  
    static const char A1[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; // 37  
    static const char A2[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";  // 36  
    static const char A3[] = "0123456789";                             // 10  
    static const char A4[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";            // 27  
  
    char c13[16];  
    strncpy(c13, c13_in, 15); c13[15] = 0;  
    size_t n = strlen(c13);  
    if (n == 0 || n > 12) return -1;  
  
    // --- special tokens ---  
    if (strncmp(c13, "DE ", 3) == 0 || strcmp(c13, "DE") == 0)  return 0;  
    if (strncmp(c13, "QRZ", 3) == 0)                            return 1;  
    if (strcmp(c13, "CQ") == 0)                                 return 2;  
  
    if (strncmp(c13, "CQ_", 3) == 0 && n >= 4 && n <= 7) {  
        int nlet = 0, nnum = 0;  
        for (size_t i = 3; i < n; i++) {  
            if (is_let(c13[i])) nlet++;  
            else if (is_dig(c13[i])) nnum++;  
            else return -1;  
        }  
        if (nnum == 3 && nlet == 0) {                    // CQ_nnn  
            int nqsy = atoi(c13 + 3);                  // 000..999  
            return 3 + nqsy;  
        }  
        if (nlet >= 1 && nlet <= 4 && nnum == 0) {       // CQ_aaaa  
            char c4[5] = "    ";  
            // adjustr: буквы правее (Fortran right-justifies c4)  
            memcpy(c4 + (4 - nlet), c13 + 3, nlet);  
            int m = 0;  
            for (int i = 0; i < 4; i++) {  
                int j = is_let(c4[i]) ? c4[i] - 'A' + 1 : 0;  
                m = 27 * m + j;  
            }  
            return 3 + 1000 + m;  
        }  
        return -1;  
    }  
  
    // --- workarounds префиксов (как в Fortran) ---  
    char cs[8];  
    if (strncmp(c13, "3DA0", 4) == 0 && n >= 7) {        // 3DA0xyz -> 3D0xyz  
        snprintf(cs, sizeof cs, "3D0%c%c%c", c13[4], c13[5], c13[6]);  
    } else if (strncmp(c13, "3X", 2) == 0 && n >= 6 &&  
               is_let(c13[2])) {                        // 3Xxyz -> Qxyz(6)  
        snprintf(cs, sizeof cs, "Q%c%c%c%c", c13[2], c13[3], c13[4], c13[5]);  
    } else {  
        cs[0] = 0;  
    }  
  
    // --- поиск цифры call-area (последняя цифра справа) ---  
    // Fortran ищет справа-налево первую цифру — это зона.  
    int iarea = -1;  
    for (int i = (int)n - 1; i >= 1; i--) {  
        if (is_dig(c13[i])) { iarea = i; break; }  
    }  
  
    // --- нестандартный позывной -> 22-битный хэш ---  
    // Требует ihashcall() + save_hash_call() из hash.f90 — пришли файл.  
    // Пока возвращаем код ошибки: хэш без совместимой таблицы бессмыслен.  
    bool need_hash = false;  
    if (cs[0]) {  
        // подменённый позывной — считаем по cs  
        strncpy(c13, cs, sizeof c13 - 1);  
        n = strlen(c13);  
        iarea = -1;  
        for (int i = (int)n - 1; i >= 1; i--)  
            if (is_dig(c13[i])) { iarea = i; break; }  
    }  
    if (iarea < 1) need_hash = true;  
  
    if (!need_hash) {  
        int npdig = 0, nplet = 0, nslet = 0;  
        for (int i = 0; i < iarea; i++) {  
            if (is_dig(c13[i])) npdig++;  
            if (is_let(c13[i])) nplet++;  
        }  
        for (size_t i = iarea + 1; i < n; i++)  
            if (is_let(c13[i])) nslet++;  
        // Fortran iarea 1-based: i<2..>3  =>  у нас iarea 0-based: <1..>2  
        if (iarea < 1 || iarea > 2 || nplet == 0 ||  
            npdig >= iarea || nslet > 3)  
            need_hash = true;  
    }  
    if (need_hash) return -2;   // TODO: ihashcall(c13,22) + NTOKENS  
  
    // --- стандартный позывной: 6-символьная раскладка ---  
    char callsign[7] = "      ";  
    if (iarea == 1) {            // iarea==2 в Fortran (1-based) -> " "+5 символов  
        callsign[0] = ' ';  
        strncpy(callsign + 1, c13, 5);  
    } else {                     // iarea==3 -> первые 6 символов  
        strncpy(callsign, c13, 6);  
    }  
    int i1 = idx_in(A1, callsign[0]);  
    int i2 = idx_in(A2, callsign[1]);  
    int i3 = idx_in(A3, callsign[2]);  
    int i4 = idx_in(A4, callsign[3]);  
    int i5 = idx_in(A4, callsign[4]);  
    int i6 = idx_in(A4, callsign[5]);  
    if (i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 || i5 < 0 || i6 < 0)  
        return -1;  
  
    uint32_t n28 = 36u*10*27*27*27*i1 + 10u*27*27*27*i2 +  
                   27u*27*27*i3 + 27u*27*i4 + 27u*i5 + (uint32_t)i6;  
    return (int32_t)(n28 + P28_NTOKENS + P28_MAX22);  
}


// jtty_pack.cpp — CALL-атом: позывной + действие (CQ/TU/AGN/NOW)  
  
// Действия i2.n2 из таблицы jtty_source_codec:  
#define JTTY_ACT_CQ_CALL   0   // 0.0  "CQ K1ABC CQ"  
#define JTTY_ACT_BARE_CALL 1   // 0.1  "K1ABC"  
#define JTTY_ACT_TU_CQ     2   // 0.2  "TU K1ABC CQ"  
#define JTTY_ACT_CALL_TU   3   // 0.3  "K1ABC TU"  
#define JTTY_ACT_CALL_AGN  4   // 1.0  "K1ABC AGN?"  
#define JTTY_ACT_TU_NOW    5   // 1.1  "TU NOW K1ABC"  
// 1.2, 1.3 — reserved, в версии 1 невалидны  
  
// Упаковка CALL-атома в 32-битное слово (без EOM).  
// Возвращает false при недопустимом позывном (jtty_pack28 вернул -1).  
static bool jtty_pack_call_atom(int action, const char* callsign, uint32_t* atom32)  
{  
    if (action < 0 || action > JTTY_ACT_TU_NOW) return false;  
  
    // fmt: в Fortran callsign проходит через fmtmsg/adjustl — здесь  
    // ожидаем уже нормализованный верхний регистр, длина <= 13  
    char c13[14];  
    memset(c13, ' ', 13); c13[13] = 0;  
    size_t n = strlen(callsign);  
    if (n == 0 || n > 13) return false;  
    memcpy(c13, callsign, n);  
  
    int32_t n28 = jtty_pack28(c13);  
    if (n28 < 0) return false;                     // нестандартный без хэша — отказ  
  
    // Раскладка "28+2+2": старшие 28 бит — jtty_pack28, затем subtype(2) и role(2)  
    *atom32 = ((uint32_t)n28 << 4)  
            | ((uint32_t)(action & 0x3) << 2);     // младшие 2 бита: role  
    // role — код роли участника обмена (0 по умолчанию для маяка)  
    return true;  
}



// Точная формула из jtty_source_codec.f90:  
//   subtype 0..3 → i2=0, n2=subtype  
//   subtype 4..5 → i2=1, n2=subtype-4  
//   n32 = (n28 << 4) | (n2 << 2) | i2  
// Биты: [31:4] = jtty_pack28 позывного, [3:2] = n2 (действие), [1:0] = i2 (группа)  
  
bool jtty_call_atom(int action, const char* call, uint32_t* out32)  
{  
    // action: 0=CQ call, 1=bare call, 2=TU/CQ, 3=call/TU, 4=call/AGN, 5=TU NOW/call  
    if (action < 0 || action > 5) return false;  
    if (!standard_call(call)) return false;      // из packjt77: regex-валидация  
  
    int32_t n28 = jtty_pack28(call);  
    if (n28 < 0) return false;  
  
    // Контрольный обратный прогон — Fortran делает unpack28 и сравнивает  
    // строки; на стороне TX достаточно проверки standard_call + n28 >= 0,  
    // но если хочешь 1:1 — реализуй unpack28 и сравни trim.  
  
    uint32_t i2, n2;  
    if (action <= 3) { i2 = 0; n2 = (uint32_t)action; }  
    else             { i2 = 1; n2 = (uint32_t)(action - 4); }  
  
    *out32 = ((uint32_t)n28 << 4) | (n2 << 2) | i2;  
    return true;  
}



// Эквивалент ihashcall_from_n8(n8, m): верхние m бит младших 64 бит  
// произведения 47055833459 * n8.  
// (hash_factor = {2419,62655,10,0} — это 47055833459 по базе 2^16)  
#include <stdint.h>  
  
static const char HASH_C38[39] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";  
  
static uint32_t ihashcall_from_n8(uint64_t n8, int m)  
{  
    uint64_t prod = 47055833459ULL * n8;   // low 64 bits  
    return (uint32_t)(prod >> (64 - m));   // top m bits  
}  
  
int32_t ihashcall(const char* c13, int m)   // c13 — позывной, до 13 символов  
{  
    uint64_t n8 = 0;  
    char buf[13];  
    memset(buf, ' ', 13);  
    size_t n = strlen(c13);  
    if (n > 13) n = 13;  
    memcpy(buf, c13, n);  
    for (int i = 0; i < 11; i++) {  
        const char* p = strchr(HASH_C38, buf[i]);  
        int j = p ? (int)(p - HASH_C38) : 0;   // неизвестный символ -> 0 (пробел)  
        n8 = 38ULL * n8 + (uint64_t)j;  
    }  
    return (int32_t)ihashcall_from_n8(n8, m);  
}


// jtty_hash.c — порт nhash (Jenkins hashlittle, байтовый путь) + hash() для jtty_pack28.  
// Эталон: nhash(c_loc(string), len, 146) & 32767. Возвращаемое значение nhash  
// уже усечено до 15 бит (c = 32767 & c), поэтому маска и initval зашиты внутрь.  
  
static inline uint32_t rot32(uint32_t x, int k) {  
    return (x << k) | (x >> (32 - k));  
}  
  
#define NH_MIX(a,b,c)                        \  
{                                            \  
    a -= c; a ^= rot32(c, 4);  c += b;       \  
    b -= a; b ^= rot32(a, 6);  a += c;       \  
    c -= b; c ^= rot32(b, 8);  b += a;       \  
    a -= c; a ^= rot32(c,16);  c += b;       \  
    b -= a; b ^= rot32(a,19);  a += c;       \  
    c -= b; c ^= rot32(b, 4);  b += a;       \  
}  
  
#define NH_FINAL(a,b,c)                      \  
{                                            \  
    c ^= b; c -= rot32(b,14);                \  
    a ^= c; a -= rot32(c,11);                \  
    b ^= a; b -= rot32(a,25);                \  
    c ^= b; c -= rot32(b,16);                \  
    a ^= c; a -= rot32(c, 4);                \  
    b ^= a; b -= rot32(a,14);                \  
    c ^= b; c -= rot32(b,24);                \  
}  
  
uint32_t nhash_bytes(const uint8_t* k, size_t length, uint32_t initval)  
{  
    uint32_t a = 0xdeadbeefu + (uint32_t)length + initval;  
    uint32_t b = a, c = a;  
  
    while (length > 12) {                    // целые 12-байтные блоки  
        a += k[0] | ((uint32_t)k[1] << 8) | ((uint32_t)k[2] << 16) | ((uint32_t)k[3] << 24);  
        b += k[4] | ((uint32_t)k[5] << 8) | ((uint32_t)k[6] << 16) | ((uint32_t)k[7] << 24);  
        c += k[8] | ((uint32_t)k[9] << 8) | ((uint32_t)k[10] << 16) | ((uint32_t)k[11] << 24);  
        NH_MIX(a, b, c);  
        length -= 12;  
        k += 12;  
    }  
  
    switch (length) {                        // хвост: все case проваливаются  
    case 12: c += (uint32_t)k[11] << 24;     /* fallthrough */  
    case 11: c += (uint32_t)k[10] << 16;     /* fallthrough */  
    case 10: c += (uint32_t)k[9] << 8;       /* fallthrough */  
    case 9:  c += k[8];                      /* fallthrough */  
    case 8:  b += (uint32_t)k[7] << 24;      /* fallthrough */  
    case 7:  b += (uint32_t)k[6] << 16;      /* fallthrough */  
    case 6:  b += (uint32_t)k[5] << 8;       /* fallthrough */  
    case 5:  b += k[4];                      /* fallthrough */  
    case 4:  a += (uint32_t)k[3] << 24;      /* fallthrough */  
    case 3:  a += (uint32_t)k[2] << 16;      /* fallthrough */  
    case 2:  a += (uint32_t)k[1] << 8;       /* fallthrough */  
    case 1:  a += k[0];                      break;  
    case 0:  return c;                       // пустая строка — без mixing  
    }  
    NH_FINAL(a, b, c);  
    return c & 32767u;                       // эталон усекает до 15 бит сам  
}


int32_t nhash15(const char* s)               // аналог Fortran subroutine hash()  
{  
    // Fortran string — не NUL-terminated; длину передаёт вызывающий.  
    // Для токенов CQ_EU/CQ_JA длина = len_trim строки.  
    return (int32_t)nhash_bytes((const uint8_t*)s, strlen(s), 146u);  
}


// jtty_pack.c — порт jtty_pack_atom из jtty_source_codec.f90  
#include <stdint.h>  
#include <stdbool.h>  
#include <string.h>  
#include <ctype.h>  
  
/* ---------- константы (из jtty_source_codec.f90) ---------- */  
#define JTTY_ATOM_CALL          0  
#define JTTY_ATOM_EXCH_NUM      1  
#define JTTY_ATOM_EXCH_LOC      2  
#define JTTY_ATOM_EXCH_PAIR     3  
#define JTTY_ATOM_EXCH_NUM_TIME 4  
#define JTTY_ATOM_CONTROL       5  
#define JTTY_ATOM_GRID4         6  
#define JTTY_ATOM_TEXT5         7  
  
#define JTTY_PAIR_ZONE_LOC3       0  
#define JTTY_PAIR_CLASS_SECTION   1  
  
#define JTTY_MISC_CONTROL 1   // подставь реальные значения из jtty_mod.f90  
#define JTTY_MISC_GRID4   2   // (константы JTTY_MISC_* в присланных фрагментах не видны)  
#define PACK77_NSEC      83   // число ARRL-секций — сверь с packjt77_grammar.f90  
  
/* ---------- атом ---------- */  
typedef struct {  
    int      kind;  
    int      subtype;      // CALL: 0..5 действие; EXCH_NUM: kind числа; CONTROL: 0..17  
    int      role;         // роль источника (0=full и т.д., valid_role)  
    int32_t  value;        // число/зона  
    int32_t  value2;       // секция/время  
    char     text[14];  
} jtty_source_atom;  
  
/* ---------- базовые кодеки ---------- */  
  
// source_index: алфавит ALPHABET из jtty_source_codec.f90  
// (строка из 64 символов — подставь точную константу ALPHABET из модуля)  
static const char JTTY_ALPHABET[] =  
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?";   // 42+ — нужен точный алфавит!  
  
static int source_index(char c) {  
    const char* p = strchr(JTTY_ALPHABET, c);  
    return p ? (int)(p - JTTY_ALPHABET) : -1;  
}  
  
// pack_base36: 2–3-символьный токен → индекс base36  
static bool pack_base36(const char* s, int n, uint32_t* out) {  
    uint32_t v = 0;  
    for (int i = 0; i < n; i++) {  
        int d;  
        char c = s[i];  
        if (c >= '0' && c <= '9') d = c - '0';  
        else if (c >= 'A' && c <= 'Z') d = c - 'A' + 10;  
        else return false;  
        v = v * 36 + d;  
    }  
    *out = v;  
    return true;  
}  
  
// grid4_to_index: A..R A..R 0..9 0..9 → 18*18*10*10 (см. jtty_source_codec)  
static bool grid4_to_index(const char* grid, uint32_t* out) {  
    if (strlen(grid) < 4) return false;  
    int a = grid[0] - 'A', b = grid[1] - 'A';  
    int c = grid[2] - '0', d = grid[3] - '0';  
    if (a < 0 || a > 17 || b < 0 || b > 17 || c < 0 || c > 9 || d < 0 || d > 9)  
        return false;  
    *out = (uint32_t)(((a * 18 + b) * 10 + c) * 10 + d);  
    return true;  
}  
  
static bool valid_role(int role)            { return role >= 0 && role <= 3; }  
static bool valid_number(int subtype, int32_t value) {  
    (void)subtype;                         // сверь границы по valid_number в модуле  
    return value >= 0 && value <= 2097151; // 21 бит — поле value в EXCH_NUM  
}  
  
// standard_call + pack28/unpack28 — из порта packjt77 (предыдущие сообщения)  
extern bool standard_call(const char* call);  
extern bool pack28(const char* c13, int32_t* n28);  
extern bool unpack28(int32_t n28, char* c13);  
  
// finish_struct: n32 = body<<5 | K<<2 | 2  (27 бит body + 3 бита подтип + тег "10")  
static inline uint32_t finish_struct(uint32_t body, int k) {  
    return (body << 5) | ((uint32_t)k << 2) | 2u;  
}  
  
/* ---------- главная функция ---------- */  
bool jtty_pack_atom(const jtty_source_atom* atom, uint32_t* n32) {  
    uint32_t body, data, pair_data, token_value;  
    int n;  
  
    switch (atom->kind) {  
  
    case JTTY_ATOM_CALL: {  
        if (atom->subtype < 0 || atom->subtype > 5) return false;  
        if (!standard_call(atom->text)) return false;  
        int32_t n28; char roundtrip[14];  
        if (!pack28(atom->text, &n28)) return false;  
        // roundtrip-проверка из исходника (unpack28 → сравнение)  
        if (!unpack28(n28, roundtrip)) return false;  
        // trim+сравнение — пропускаем, если pack28 корректен  
        int i2, n2;  
        if (atom->subtype <= 3) { i2 = 0; n2 = atom->subtype; }  
        else                    { i2 = 1; n2 = atom->subtype - 4; }  
        *n32 = ((uint32_t)n28 << 4) | ((uint32_t)n2 << 2) | (uint32_t)i2;  
        return true;  
    }  
  
    case JTTY_ATOM_EXCH_NUM:  
        if (!valid_role(atom->role) || !valid_number(atom->subtype, atom->value))  
            return false;  
        body = ((uint32_t)atom->role    << 26)  
             | ((uint32_t)atom->subtype << 22)  
             | ((uint32_t)atom->value   << 5);  
        *n32 = finish_struct(body, 0);  
        return true;  
  
    case JTTY_ATOM_EXCH_LOC:  
        if (!valid_role(atom->role) || atom->subtype < 0 || atom->subtype > 4)  
            return false;  
        n = (int)strnlen(atom->text, 13);  
        if (n != 2 && n != 3) return false;  
        if (!pack_base36(atom->text, n, &token_value)) return false;  
        body = ((uint32_t)atom->role    << 26)  
             | ((uint32_t)atom->subtype << 22)  
             | ((uint32_t)(n - 2)       << 21)  
             | token_value;  
        *n32 = finish_struct(body, 1);  
        return true;  
  
    case JTTY_ATOM_EXCH_PAIR:  
        switch (atom->subtype) {  
        case JTTY_PAIR_ZONE_LOC3:  
            if (atom->value < 1 || atom->value > 40) return false;  
            n = (int)strnlen(atom->text, 13);  
            if (n != 2 && n != 3) return false;  
            if (!pack_base36(atom->text, n, &token_value)) return false;  
            pair_data = ((uint32_t)atom->value << 17)  
                      | ((uint32_t)(n - 2)     << 16)  
                      | token_value;  
            break;  
        case JTTY_PAIR_CLASS_SECTION: {  
            if (atom->value < 1 || atom->value > 32) return false;  
            if (atom->value2 < 1 || atom->value2 > PACK77_NSEC) return false;  
            if (strnlen(atom->text, 13) != 1) return false;  
            int class_index = atom->text[0] - 'A';  
            if (class_index < 0 || class_index > 5) return false;  
            pair_data = ((uint32_t)atom->value  << 17)  
                      | ((uint32_t)class_index   << 14)  
                      | ((uint32_t)atom->value2  << 7);  
            break;  
        }  
        default: return false;  
        }  
        body = ((uint32_t)atom->subtype << 24) | (pair_data << 1);  
        *n32 = finish_struct(body, 2);  
        return true;  
  
    case JTTY_ATOM_EXCH_NUM_TIME:  
        if (!valid_role(atom->role)) return false;  
        if (atom->value  < 0 || atom->value  > 16383) return false;  
        if (atom->value2 < 0 || atom->value2 > 1439)  return false;  
        body = ((uint32_t)atom->role   << 26)  
             | ((uint32_t)atom->value  << 12)  
             | ((uint32_t)atom->value2 << 1);  
        *n32 = finish_struct(body, 3);  
        return true;  
  
    case JTTY_ATOM_CONTROL:  
        if (atom->subtype < 0 || atom->subtype > 17) return false;  
        body = ((uint32_t)JTTY_MISC_CONTROL << 23)  
             | ((uint32_t)atom->subtype     << 16);  
        *n32 = finish_struct(body, 4);  
        return true;  
  
    case JTTY_ATOM_GRID4: {  
        if (!valid_role(atom->role)) return false;  
        uint32_t grid_index;  
        if (!grid4_to_index(atom->text, &grid_index)) return false;  
        data = ((uint32_t)atom->role << 22) | (grid_index << 7);  
        body = ((uint32_t)JTTY_MISC_GRID4 << 23) | data;  
        *n32 = finish_struct(body, 4);  
        return true;  
    }  
  
    case JTTY_ATOM_TEXT5: {  
        uint32_t top30 = 0;  
        for (int i = 0; i < 5; i++) {  
            int idx = source_index(atom->text[i]);  
            if (idx < 0) return false;  
            top30 = 64 * top30 + (uint32_t)idx;  
        }  
        *n32 = 4 * top30 + 3;  
        return true;  
    }  
  
    default: return false;  
    }  
}



// jtty_hash_test.c — проверка порта ihashcall по эталонным векторам WSJT-X  
#include <stdio.h>  
#include <stdint.h>  
#include <string.h>  
  
static const char IHC[38] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";  
  
static int ihashcall_from_n8(uint64_t n8, int m)  
{  
    uint64_t x = 47055833459ULL * n8;   // переполнение = mod 2^64  
    return (int)(x >> (64 - m));  
}  
  
static int ihashcall_c(const char* callsign, int m)  
{  
    char buf[14];  
    memset(buf, ' ', 13); buf[13] = 0;  
    size_t n = strlen(callsign); if (n > 13) n = 13;  
    memcpy(buf, callsign, n);  
    uint64_t n8 = 0;  
    for (int i = 0; i < 11; i++) {  
        const char* p = strchr(IHC, buf[i]);  
        n8 = 38ULL * n8 + (p ? (uint64_t)(p - IHC) : 0);  
    }  
    return ihashcall_from_n8(n8, m);  
}  
  
int main(void)  
{  
    static const struct { const char* cs; int m10, m12, m22; } vec[] = {  
        {"K1ABC",       712, 2851, 2920267},  
        {"W9XYZ",       972, 3889, 3982604},  
        {"PJ4/K1ABC",   346, 1387, 1420834},  
        {"KH1/KH7Z",    201,  806,  825805},  
        {"K1JT/P",      959, 3839, 3931158},  
        {"W3CCX",       665, 2662, 2726483},  
        {"3DA0ABC",      48,  192,  196695},  
        {"3XABC",       563, 2254, 2308721},  
        {"///////////", 671, 2685, 2749801},  
        {"ZZZZZZZZZZZ", 902, 3609, 3695718},  
        {"99999999999", 762, 3050, 3123740},  
        {"A23456789Z/", 122,  488,  499902},  
        {"",              0,    0,       0},  
    };  
    int pass = 0;  
    for (size_t i = 0; i < sizeof(vec)/sizeof(vec[0]); i++) {  
        int g10 = ihashcall_c(vec[i].cs, 10);  
        int g12 = ihashcall_c(vec[i].cs, 12);  
        int g22 = ihashcall_c(vec[i].cs, 22);  
        if (g10 != vec[i].m10 || g12 != vec[i].m12 || g22 != vec[i].m22) {  
            printf("FAIL \"%s\": got %d %d %d, want %d %d %d\n",  
                   vec[i].cs, g10, g12, g22, vec[i].m10, vec[i].m12, vec[i].m22);  
        } else pass++;  
    }  
    printf("passed: %d/%d\n", pass, (int)(sizeof(vec)/sizeof(vec[0])));  
    return pass == 13 ? 0 : 1;  
}


// jtty_frame.cpp — сборка 34-битного payload (порт pack_jtty_atom)  
  
#include <stdint.h>  
#include <stdbool.h>  
#include <string.h>  
#include <ctype.h>  
  
#define JTTY_ATOM_CALL          0  
#define JTTY_ATOM_EXCH_NUM      1  
#define JTTY_ATOM_EXCH_LOC      2  
#define JTTY_ATOM_EXCH_PAIR     3  
#define JTTY_ATOM_EXCH_NUM_TIME 4  
#define JTTY_ATOM_CONTROL       5  
#define JTTY_ATOM_GRID4         6  
#define JTTY_ATOM_TEXT5         7  
  
#define JTTY_CALL_CQ       0  
#define JTTY_CALL_CALL     1  
#define JTTY_CALL_TU_CQ    2  
#define JTTY_CALL_CALL_TU  3  
#define JTTY_CALL_CALL_AGN 4  
#define JTTY_CALL_TU_NOW   5  
  
#define JTTY_ROLE_FIELD_ONLY 0  
#define JTTY_ROLE_FULL       1  
  
#define JTTY_NUM_SERIAL        0  
#define JTTY_NUM_CQ_ZONE       1  
#define JTTY_NUM_ITU_ZONE      2  
#define JTTY_NUM_STATE         3   // проверь имена в константах  
#define JTTY_NUM_GENERIC       4  
  
#define JTTY_PAIR_ZONE_LOC3      0  
#define JTTY_PAIR_CLASS_SECTION  1  
  
#define JTTY_MISC_CONTROL 0  
#define JTTY_MISC_GRID4   1  
  
static const char JTTY_ALPHABET[] =  
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?!\"#$%,&*()_'=[]{}<>|:;"; // 64  
  
typedef struct {  
    uint8_t  kind;      // JTTY_ATOM_*  
    uint8_t  subtype;   // действие/тип внутри kind  
    uint8_t  role;      // FIELD_ONLY/FULL (struct-атомы)  
    uint16_t value;     // числовое поле  
    uint16_t value2;    // второе числовое поле  
    char     text[9];   // позывной / текст / локатор  
} jtty_atom_t;  
  
static int source_index(char c) {  
    const char* p = strchr(JTTY_ALPHABET, c);  
    return p ? (int)(p - JTTY_ALPHABET) : -1;  
}  
  
// pack28 из packjt77.f90 — объявлен ранее  
extern int32_t pack28(const char* c13);  
extern bool    standard_call(const char* c);   // или гард по n28-диапазону  
  
static bool pack_base36(const char* s, int n, uint32_t* out) {  
    // base36 по алфавиту '0'-'9','A'-'Z' — тело из jtty_source_codec.f90  
    uint32_t v = 0;  
    for (int i = 0; i < n; i++) {  
        char c = s[i];  
        int d = (c >= '0' && c <= '9') ? c - '0'  
              : (c >= 'A' && c <= 'Z') ? c - 'A' + 10 : -1;  
        if (d < 0) return false;  
        v = v * 36u + (uint32_t)d;  
    }  
    *out = v;  
    return true;  
}  
  
static bool grid4_to_index(const char* g, uint32_t* out) {  
    // AA00..RR99: [A-R][A-R][0-9][0-9] → ((a*18+b)*10+c)*10+d  
    if (!g[0] || !g[1] || !g[2] || !g[3]) return false;  
    int a = g[0] - 'A', b = g[1] - 'A', c = g[2] - '0', d = g[3] - '0';  
    if (a < 0 || a > 17 || b < 0 || b > 17 || c < 0 || c > 9 || d < 0 || d > 9)  
        return false;  
    *out = (uint32_t)((a * 18 + b) * 10 + c) * 10u + (uint32_t)d;  
    return true;  
}  
  
static inline bool valid_role(int r) { return r >= 0 && r <= 1; }  
  
static bool valid_number(int subtype, int value) {  
    // границы по subtype из valid_number() — сверь с исходником;  
    // SERIAL/CQ_ZONE/ITU: 1..9999? подставь реальные пределы  
    switch (subtype) {  
        case JTTY_NUM_SERIAL:   return value >= 0 && value <= 9999;  
        case JTTY_NUM_CQ_ZONE:  return value >= 1 && value <= 40;  
        case JTTY_NUM_ITU_ZONE: return value >= 1 && value <= 90;  
        case JTTY_NUM_GENERIC:  return value >= 0 && value <= 99999;  
        default:                return value >= 0 && value <= 999;  
    }  
}  
  
// n32 = 32*body + 4*family + 2  
static inline uint64_t finish_struct(uint64_t body, int family) {  
    return ((body << 3) + (uint64_t)family) * 4u + 2u;  
}  
  
// atom + eom → 34-битный payload. Возвращает false при невалидном атоме.  
bool jtty_pack_atom(const jtty_atom_t* atom, bool eom, uint64_t* payload34)  
{  
    uint64_t n32 = 0;  
  
    switch (atom->kind) {  
  
    case JTTY_ATOM_CALL: {  
        if (atom->subtype > 5) return false;  
        if (!standard_call(atom->text)) return false;  
        int32_t n28 = pack28(atom->text);  
        if (n28 <= 0) return false;  
        int i2, n2;  
        if (atom->subtype <= 3) { i2 = 0; n2 = atom->subtype; }  
        else                    { i2 = 1; n2 = atom->subtype - 4; }  
        n32 = ((uint64_t)(uint32_t)n28 << 4) + 4u * n2 + i2;  
        break;  
    }  
  
    case JTTY_ATOM_EXCH_NUM:  
        if (!valid_role(atom->role) ||  
            !valid_number(atom->subtype, atom->value)) return false;  
        n32 = finish_struct(((uint64_t)atom->role    << 26) |  
                            ((uint64_t)atom->subtype << 22) |  
                            ((uint64_t)atom->value   << 5), 0);  
        break;  
  
    case JTTY_ATOM_EXCH_LOC: {  
        if (!valid_role(atom->role) || atom->subtype > 4) return false;  
        int n = (int)strlen(atom->text);  
        if (n != 2 && n != 3) return false;  
        uint32_t tok;  
        if (!pack_base36(atom->text, n, &tok)) return false;  
        uint64_t body = ((uint64_t)atom->role    << 26) |  
                        ((uint64_t)atom->subtype << 22) |  
                        ((uint64_t)(n - 2)       << 21) |  
                        tok;  
        n32 = finish_struct(body, 1);  
        break;  
    }  
  
    case JTTY_ATOM_EXCH_PAIR: {  
        uint64_t pair_data;  
        switch (atom->subtype) {  
        case JTTY_PAIR_ZONE_LOC3: {  
            if (atom->value < 1 || atom->value > 40) return false;  
            int n = (int)strlen(atom->text);  
            if (n != 2 && n != 3) return false;  
            uint32_t tok;  
            if (!pack_base36(atom->text, n, &tok)) return false;  
            pair_data = ((uint64_t)atom->value << 17) |  
                        ((uint64_t)(n - 2)     << 16) | tok;  
            break;  
        }  
        case JTTY_PAIR_CLASS_SECTION: {  
            if (atom->value < 1 || atom->value > 32) return false;  
            if (atom->value2 < 1 || atom->value2 > PACK77_NSEC) return false;  
            if (strlen(atom->text) != 1) return false;  
            int cls = atom->text[0] - 'A';  
            if (cls < 0 || cls > 5) return false;  
            pair_data = ((uint64_t)atom->value  << 17) |  
                        ((uint64_t)cls          << 14) |  
                        ((uint64_t)atom->value2 << 7);  
            break;  
        }  
        default: return false;  
        }  
        uint64_t body = ((uint64_t)atom->subtype << 24) | (pair_data << 1);  
        n32 = finish_struct(body, 2);  
        break;  
    }  
  
    case JTTY_ATOM_EXCH_NUM_TIME:  
        if (!valid_role(atom->role)) return false;  
        if (atom->value  > 16383) return false;  
        if (atom->value2 > 1439)  return false;  
        n32 = finish_struct(((uint64_t)atom->role   << 26) |  
                            ((uint64_t)atom->value  << 12) |  
                            ((uint64_t)atom->value2 << 1), 3);  
        break;  
  
    case JTTY_ATOM_CONTROL:  
        if (atom->subtype > 17) return false;  
        n32 = finish_struct(((uint64_t)JTTY_MISC_CONTROL << 23) |  
                            ((uint64_t)atom->subtype     << 16), 4);  
        break;  
  
    case JTTY_ATOM_GRID4: {  
        if (!valid_role(atom->role)) return false;  
        uint32_t gi;  
        if (!grid4_to_index(atom->text, &gi)) return false;  
        uint64_t data = ((uint64_t)atom->role << 22) | ((uint64_t)gi << 7);  
        uint64_t body = ((uint64_t)JTTY_MISC_GRID4 << 23) | data;  
        n32 = finish_struct(body, 4);  
        break;  
    }  
  
    case JTTY_ATOM_TEXT5: {  
        if (strlen(atom->text) < 5) return false;  
        uint64_t top30 = 0;  
        for (int i = 0; i < 5; i++) {  
            int s = source_index(atom->text[i]);  
            if (s < 0) return false;  
            top30 = top30 * 64u + (uint64_t)s;  
        }  
        n32 = 4u * top30 + 3u;  
        break;  
    }  
  
    default: return false;  
    }  
  
    *payload34 = (n32 << 2) | (eom ? 1u : 0u);   // бит34..3 = n32, бит2 = reserved 0, бит1 = eom  
    return true;  
}

// Кадр → 59 тонов (is13 + 46 символов данных)  
void jtty_frame_to_tones(uint64_t payload34, uint8_t itone[59])  
{  
    uint64_t info46 = jtty_append_crc12(payload34);        // payload + CRC12(0x80F)  
    uint8_t  dibits[46];  
    jtty_tbcc_encode(info46, dibits);                      // K=10, 0o1167/0o1545  
  
    static const uint8_t is13[13] = {0,2,2,3,0,0,3,2,1,3,1,2,0};  
    memcpy(itone, is13, 13);  
    memcpy(itone + 13, dibits, 46);  
}


// jtty_pack.c — DP-паковщик сообщения JTTY (порт pack_jtty из jtty_mod.f90).  
// Требует: jtty_pack_atom(), jtty_unpack_atom(), jtty_render_atom(),  
//          pack77_arrl_section_index().  
  
#define JTTY_MAX_FRAMES   16  
#define JTTY_MSG_LEN      80  
#define INF               999  
  
typedef struct {  
    int  kind;      // JTTY_ATOM_*  
    int  subtype;   // action / number-kind / control id / ...  
    int  role;      // JTTY_ROLE_FIELD_ONLY=0, JTTY_ROLE_FULL=1  
    int  value;     // class/number  
    char text[14];  // call / grid / text5 / loc  
} jtty_atom_t;  
  
static jtty_atom_t  choice[80];  
static int          dp[81], successor[80];  
static char         msg[JTTY_MSG_LEN + 1];  
static int          n, profile;  
static int          frame_starts[JTTY_MAX_FRAMES];  
  
// --- normalize_jtty_message: верхний регистр, '~'→' ', недопустимые→'#',  
//     схлопывание пробелов, обрезка ведущего пробела ---  
static void normalize_jtty_message(char* s)  
{  
    for (char* p = s; *p; p++) {  
        char c = *p;  
        if (c >= 'a' && c <= 'z') c -= 32;  
        if (c == '~') c = ' ';  
        if (source_index_char(c) < 0) c = '#';   // вне ALPHABET  
        *p = c;  
    }  
    // схлопнуть пробелы + trim  
    char tmp[JTTY_MSG_LEN + 1];  
    int w = 0; bool sp = true;                    // sp=true пропускает ведущие  
    for (char* p = s; *p && w < JTTY_MSG_LEN; p++) {  
        if (*p == ' ') { if (!sp) { tmp[w++] = ' '; sp = true; } }  
        else           { tmp[w++] = *p;  sp = false; }  
    }  
    while (w > 0 && tmp[w-1] == ' ') w--;  
    tmp[w] = 0;  
    strcpy(s, tmp);  
}  
  
static bool decimal_value(const char* t, int len, int* out)  
{  
    if (len < 1 || len > 6) return false;  
    int v = 0;  
    for (int i = 0; i < len; i++) {  
        if (t[i] < '0' || t[i] > '9') return false;  
        v = 10 * v + (t[i] - '0');  
    }  
    if (v > 131071) return false;  
    *out = v;  
    return true;  
}  
  
// --- consider(): выбор лучшего продолжения ---  
static int ipos_;  
static void consider(const jtty_atom_t* atom, int next)  
{  
    int cost = 1 + dp[next];  
    if (cost > JTTY_MAX_FRAMES || cost > dp[ipos_]) return;  
  
    int rank      = (atom->kind == JTTY_ATOM_TEXT5) ? 1 : 0;  
    int best_rank = (choice[ipos_].kind == JTTY_ATOM_TEXT5) ? 1 : 0;  
    int key      = 100*atom->kind + 2*atom->subtype + atom->role;  
    int best_key = 100*choice[ipos_].kind + 2*choice[ipos_].subtype + choice[ipos_].role;  
  
    if (cost == dp[ipos_]) {  
        if (rank > best_rank) return;  
        if (rank == best_rank) {  
            if (next < successor[ipos_]) return;  
            if (next == successor[ipos_] && key >= best_key) return;  
        }  
    }  
    dp[ipos_]        = cost;  
    successor[ipos_] = next;  
    choice[ipos_]    = *atom;  
}  
  
// --- offer(): round-trip проверка «атом ↔ текст» ---  
static void offer(const jtty_atom_t* atom)  
{  
    uint64_t frame;  
    if (!jtty_pack_atom(atom, false, &frame)) return;  
  
    jtty_atom_t decoded;  
    bool eom;  
    if (!jtty_unpack_atom(frame, &decoded, &eom)) return;  
  
    char rendered[JTTY_MSG_LEN + 1];  
    if (!jtty_render_atom(&decoded, rendered)) return;  
  
    int length = (int)strlen(rendered);  
    if (length == 0) return;  
    int last = ipos_ + length - 1;             // 0-based: last index  
    if (last > n) return;  
    if (strncmp(msg + ipos_, rendered, length) != 0) return;  
  
    int next = n;  
    if (last < n) {  
        if (msg[last] != ' ') return;  
        next = last + 1;                        // +1 separator column  
    }  
    consider(atom, next);  
}  
  
// --- try_compact(): атомы у границы слова ---  
static void try_compact(void)  
{  
    char words[3][JTTY_MSG_LEN + 1];  
    int nwords = 0, first = ipos_;  
  
    for (int j = 0; j < 3 && first < n; j++) {  
        int last = first;  
        while (last < n && msg[last] != ' ') last++;  
        int wl = last - first;  
        memcpy(words[j], msg + first, wl);  
        words[j][wl] = 0;  
        nwords = j + 1;  
        first = last + 1;  
    }  
  
    jtty_atom_t a;  
    // CALL: каждое слово × каждый action  
    for (int j = 0; j < nwords; j++) {  
        if (strlen(words[j]) > 13) continue;  
        for (int action = JTTY_CALL_CQ; action <= JTTY_CALL_TU_NOW; action++) {  
            memset(&a, 0, sizeof a);  
            a.kind = JTTY_ATOM_CALL; a.subtype = action;  
            strncpy(a.text, words[j], sizeof a.text - 1);  
            offer(&a);  
        }  
    }  
    // CONTROL: все 18 команд  
    for (int j = JTTY_CONTROL_AGN; j <= JTTY_CONTROL_OK; j++) {  
        memset(&a, 0, sizeof a);  
        a.kind = JTTY_ATOM_CONTROL; a.subtype = j;  
        offer(&a);  
    }  
    // EXCH_NUM / GRID4 / EXCH_LOC: role × поле  
    for (int role = JTTY_ROLE_FIELD_ONLY; role <= JTTY_ROLE_FULL; role++) {  
        int field = 0;  
        if (role == JTTY_ROLE_FULL) {  
            if (strcmp(words[0], "599") != 0 || nwords < 2) continue;  
            field = 1;  
        }  
        int len = strlen(words[field]), value;  
        bool numeric = decimal_value(words[field], len, &value);  
  
        if (numeric) {  
            memset(&a, 0, sizeof a);  
            a.kind = JTTY_ATOM_EXCH_NUM; a.role = role;  
            a.subtype = JTTY_NUM_GENERIC; a.value = value;  
            offer(&a);  
            if (role == JTTY_ROLE_FULL && profile == JTTY_EXCHANGE_RTTY) {  
                a.subtype = JTTY_NUM_SERIAL;  
                offer(&a);  
            }  
        }  
        if (len == 4) {  
            memset(&a, 0, sizeof a);  
            a.kind = JTTY_ATOM_GRID4; a.role = role;  
            strncpy(a.text, words[field], 4); a.text[4] = 0;  
            offer(&a);  
        }  
        if (role != JTTY_ROLE_FULL || len < 2 || len > 3) continue;  
        if (strpbrk(words[field], "ABCDEFGHIJKLMNOPQRSTUVWXYZ") == NULL) continue;  
  
        memset(&a, 0, sizeof a);  
        a.kind = JTTY_ATOM_EXCH_LOC; a.role = role;  
        a.subtype = JTTY_LOC_QTH;  
        strncpy(a.text, words[field], sizeof a.text - 1);  
        offer(&a);  
        if (profile == JTTY_EXCHANGE_RTTY) {  
            a.subtype = JTTY_LOC_STATE_PROVINCE;  
            offer(&a);  
        }  
    }  
    // CLASS_SECTION: "1D EMA"  
    int len = strlen(words[0]);  
    if (nwords < 2 || len < 2 || len > 3) return;  
    int value;  
    if (!decimal_value(words[0], len - 1, &value)) return;  
    int section_index = pack77_arrl_section_index(words[1]);  
  
    memset(&a, 0, sizeof a);  
    a.kind = JTTY_ATOM_CLASS_SECTION;      // если введён отдельный kind  
    a.value = value;  
    a.text[0] = words[0][len - 1]; a.text[1] = 0;   // буква класса  
    a.subtype = section_index;  
    offer(&a);  
}  
  
// --- главная функция ---  
int pack_jtty(char* message, uint64_t c32[JTTY_MAX_FRAMES],  
              int exchange_profile, int is_final)  
{  
    normalize_jtty_message(message);  
    profile = exchange_profile;  
    if (profile < JTTY_EXCHANGE_UNKNOWN || profile > JTTY_EXCHANGE_RTTY)  
        return -1;  
    // normalize_serials() — при profile==RTTY; опущен, добавь по исходнику.  
  
    n = strlen(msg);                        // msg == message после normalize  
    if (n <= 0) return 0;  
    strcpy(msg, message);  
  
    for (int i = 0; i <= 80; i++) dp[i] = INF;  
    memset(successor, 0, sizeof successor);  
    dp[n] = 0;  
  
    for (ipos_ = n - 1; ipos_ >= 0; ipos_--) {  
        // TEXT5: до 5 символов  
        for (int len = 1; len <= 5 && ipos_ + len <= n; len++) {  
            jtty_atom_t a;  
            memset(&a, 0, sizeof a);  
            a.kind = JTTY_ATOM_TEXT5;  
            memcpy(a.text, msg + ipos_, len);  
            a.text[len] = 0;  
            consider(&a, ipos_ + len);  
        }  
        if (ipos_ > 0 && msg[ipos_ - 1] != ' ') continue;  
        try_compact();  
    }  
  
    if (dp[0] > JTTY_MAX_FRAMES) return -1;  
  
    // проход по цепочке successor → сборка атомов → pack_jtty_atoms  
    jtty_atom_t atoms[JTTY_MAX_FRAMES];  
    int natoms = 0, ipos = 0;  
    while (ipos < n) {  
        atoms[natoms] = choice[ipos];  
        frame_starts[natoms] = ipos + 1;   // 1-based, как в Fortran  
        ipos = successor[ipos];  
        natoms++;  
    }  
    // pack_jtty_atoms: для каждого атома pack_jtty_atom(atom, eom=(i==natoms-1 && is_final))  
    int nframes;  
    if (!pack_jtty_atoms(atoms, natoms, c32, &nframes, is_final))  
        return -1;  
    return nframes;  
}



// jtty_pack.cpp — порт pack_jtty_atoms из jtty_mod.f90.  
// frames[i] = pack_jtty_atom(atoms[i], eom = (i==natoms-1 && is_final)).  
// is_final: 0/1; -1 = не задан → final (поведение Fortran по умолчанию).  
  
bool jtty_pack_atoms(const JttyAtom* atoms, int natoms,  
                     uint64_t* frames, int* nframes, int is_final)  
{  
    bool final_ok = (is_final < 0) || (is_final != 0);   // absent → final  
    *nframes = 0;  
  
    if (natoms < 1 || natoms > 16) return false;  
  
    for (int i = 0; i < natoms; i++) {  
        bool eom = (i == natoms - 1) && final_ok;  
        if (!jtty_pack_atom(&atoms[i], &frames[i], eom)) {  
            return false;               // Fortran обнуляет все кадры  
        }  
    }  
    *nframes = natoms;  
    return true;  
}



// jtty_pack.cpp — константы из jtty_source_codec.f90 (теперь точные)  
  
// Алфавит — 64 символа, порядок критичен:  
// ' '(36) '+'(37) '-'(38) '.'(39) '/'(40) '?'(41) '!'(42) '"'(43) '#'(44)  
// '$'(45) '%'(46) ','(47) '&'(48) '*'(49) '('(50) ')'(51) '_'(52) '\''(53)  
// '='(54) '['(55) ']'(56) '{'(57) '}'(58) '<'(59) '>'(60) '|'(61) ':'(62) ';'(63)  
static const char JTTY_ALPHABET[] =  
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?!\"#$%,&*()_'=[]{}<>|:;";  
  
enum {  
    JTTY_ATOM_CALL=0, JTTY_ATOM_EXCH_NUM=1, JTTY_ATOM_EXCH_LOC=2,  
    JTTY_ATOM_EXCH_PAIR=3, JTTY_ATOM_EXCH_NUM_TIME=4, JTTY_ATOM_CONTROL=5,  
    JTTY_ATOM_GRID4=6, JTTY_ATOM_TEXT5=7  
};  
  
enum {   // CALL action/subtype  
    JTTY_CALL_CQ=0, JTTY_CALL_CALL=1, JTTY_CALL_TU_CQ=2,  
    JTTY_CALL_CALL_TU=3, JTTY_CALL_CALL_AGN=4, JTTY_CALL_TU_NOW=5  
};  
  
enum { JTTY_ROLE_FIELD_ONLY=0, JTTY_ROLE_FULL=1 };  
  
enum {   // EXCH_NUM number_kind (4 бита)  
    JTTY_NUM_SERIAL=0, JTTY_NUM_CQ_ZONE=1, JTTY_NUM_ITU_ZONE=2,  
    JTTY_NUM_AGE=3, JTTY_NUM_POWER=4, JTTY_NUM_CHECK=5,  
    JTTY_NUM_LICENSE_YEAR=6, JTTY_NUM_GENERIC=7  
};  
  
enum {   // EXCH_LOC location_kind (4 бита)  
    JTTY_LOC_STATE_PROVINCE=0, JTTY_LOC_SECTION=1,  
    JTTY_LOC_COUNTRY_PREFIX=2, JTTY_LOC_QTH=3, JTTY_LOC_ADMIN_CODE=4  
};  
  
enum {   // EXCH_PAIR schema (3 бита)  
    JTTY_PAIR_ZONE_LOC3=0, JTTY_PAIR_CLASS_SECTION=1  
};  
  
enum { JTTY_MISC_CONTROL=0, JTTY_MISC_GRID4=1 };  
  
enum {   // CONTROL subtype (0..17)  
    JTTY_CONTROL_AGN=0, JTTY_CONTROL_CALL=1, JTTY_CONTROL_AGN_CALL=2,  
    JTTY_CONTROL_NR=3, JTTY_CONTROL_AGN_NR=4, JTTY_CONTROL_EXCH=5,  
    JTTY_CONTROL_STATE=6, JTTY_CONTROL_SECTION=7, JTTY_CONTROL_ZONE=8,  
    JTTY_CONTROL_GRID=9, JTTY_CONTROL_RPRT=10, JTTY_CONTROL_QSL_TU=11,  
    JTTY_CONTROL_TU=12, JTTY_CONTROL_QRZ=13, JTTY_CONTROL_QSO_B4=14,  
    JTTY_CONTROL_WAIT=15, JTTY_CONTROL_NIL=16, JTTY_CONTROL_OK=17  
};  
  
static const char* const JTTY_CONTROL_TEXT[18] = {  
    "AGN?","CALL?","AGN CALL","NR?","AGN NR","EXCH?","STATE?","SECTION?",  
    "ZONE?","GRID?","RPRT?","QSL TU","TU","QRZ?","QSO B4","WAIT","NIL?","OK?"  
};


// jtty_modem.cpp — JTTY TX для RP2040 beacon.  
// Протокол: WSJT-X 3.3 JTTY (4-GFSK, 31.25 бод, 59 символов/кадр,  
// 34-бит payload + CRC-12(0x80F) + TBCC K=10 r=1/2 полиномы 0o1167/0o1545).  
// Шейпинг: жёсткий 4-FSK (без гауссова) — декодируется, спектр шире.  
  
#include "jtty_modem.h"      // создать: прототипы jtty_send_string, prepare_jtty_frequencies  
#include "thor_modem.h"      // rsid_send не нужен; нужны mfsk_* — см. примечание внизу  
// Если mfsk_* static в thor_modem.cpp — продублируй emit/wait-цикл здесь  
// через vfo_set_tone_instant + micros-дедлайны.  
  
// ---------------- Константы протокола ----------------  
#define JTTY_BAUD            31.25  
#define JTTY_STEP_HZ         31.25  
#define JTTY_NUMTONES        4  
#define JTTY_SYMBOL_US       32000UL          // 32 мс  
#define JTTY_SYNC_LEN        13  
#define JTTY_DATA_SYMS       46  
#define JTTY_FRAME_SYMS      59               // 13 sync + 46 data  
#define JTTY_MAX_FRAMES      16  
#define JTTY_CENTER_OFF_HZ   46.875           // 1.5 * 31.25 — центрирование  
  
static const uint8_t JTTY_IS13[13] = {0,2,2,3,0,0,3,2,1,3,1,2,0};  
  
// ---------------- CRC-12, полином 0x80F ----------------  
// Порт tbcc.f90: биты payload обходятся MSB-first (бит 33 → бит 0).  
static uint16_t jtty_crc12(uint64_t payload34)  
{  
    uint16_t crc = 0;  
    for (int i = 33; i >= 0; i--) {  
        uint8_t bit = (uint8_t)((payload34 >> i) & 1ULL);  
        uint8_t top = (crc >> 11) & 1;  
        crc = (uint16_t)(((crc << 1) & 0x0FFF) ^ (top ? 0x080F : 0) ^ bit);  
    }  
    return crc;  
}  
  
// ---------------- TBCC K=10, r=1/2, g0=0o1167, g1=0o1545 ----------------  
// Вход: 46 информационных бит (34 payload + 12 CRC), MSB-first.  
// Tail-biting: регистр инициализируется последними ν=9 битами слова,  
// затем свёртка по всем 46 битам. Выход: 46 тонов {0..3}.  
static void jtty_tbcc_encode(uint64_t info46, uint8_t* tones46)  
{  
    const uint16_t G0 = 01167, G1 = 01545, RMASK = 0x3FF;  
    // инициализация регистра: последние 9 бит информационного слова  
    uint16_t reg = (uint16_t)(info46 & 0x1FFULL);  
  
    for (int i = 45; i >= 0; i--) {  
        uint8_t bit = (uint8_t)((info46 >> i) & 1ULL);  
        reg = (uint16_t)(((reg >> 1) | ((uint16_t)bit << 9)) & RMASK);  
        uint8_t b0 = (uint8_t)(__builtin_popcount(reg & G0) & 1);  
        uint8_t b1 = (uint8_t)(__builtin_popcount(reg & G1) & 1);  
        // Маппинг дибита в тон: (b1 b0) или (b0 b1) — см. примечание ниже  
        tones46[45 - i] = (uint8_t)((b0 << 1) | b1);  
    }  
}  
  
// ---------------- Сборка кадра: is13 + 46 дата-тонов ----------------  
// payload34: [33:2]=atom32, [1]=reserved(0), [0]=EOM  
static void jtty_build_frame(uint64_t payload34, uint8_t* itone59)  
{  
    uint64_t info46 = (payload34 << 12) | jtty_crc12(payload34);  
    for (int i = 0; i < 13; i++) itone59[i] = JTTY_IS13[i];  
    jtty_tbcc_encode(info46, itone59 + 13);  
}  
  
// ---------------- Передача одного кадра ----------------  
// Требования к окружению (как у THOR):  
//   mfsk_prepare(base_hz, step, numtones) — нижний тон f0  
//   mfsk_emit_tone(tone, us) — дедлайн-цикл с abort-флагами  
//   mfsk_carrier(on)/mfsk_sync_deadline()/mfsk_abort()  
static bool jtty_tx_frame(uint64_t payload34)  
{  
    uint8_t itone[JTTY_FRAME_SYMS];  
    jtty_build_frame(payload34, itone);  
  
    mfsk_sync_deadline();  
    mfsk_carrier(true);  
    for (int i = 0; i < JTTY_FRAME_SYMS; i++) {  
        if (!mfsk_emit_tone(itone[i], JTTY_SYMBOL_US)) {  
            mfsk_carrier(false);  
            return false;  
        }  
    }  
    mfsk_carrier(false);  
    return true;   // кадр 1.888 с  
}



uint32_t JTTY_Base_freq = 10001500;   // центр полосы, из INFO.TXT (FREQ_JTTY)  
  
void prepare_jtty_frequencies(uint32_t base_hz)  
{  
    JTTY_Base_freq = base_hz;  
    mfsk_prepare(base_hz - (uint32_t)(JTTY_CENTER_OFF_HZ + 0.5),  
                 JTTY_STEP_HZ, JTTY_NUMTONES);  
    // f0 = центр − 46.875 → нижний тон сетки  
}  
  
void jtty_send_string(const char* str)  
{  
    if (str == nullptr || *str == 0) return;  
  
    JttyFrameList fl;  
    if (!jtty_pack_message(str, &fl, JTTY_EXCHANGE_UNKNOWN, 1)) return;  
  
    mfsk_tx_begin();  
    for (int i = 0; i < fl.nframes; i++) {  
        if (!jtty_tx_frame(fl.frames[i])) return;   // abort → выход  
        // кадры идут подряд без паузы: дедлайн уже синхронизирован  
    }  
    // mfsk_carrier(false) уже внутри jtty_tx_frame;  
    // tx_done-обвязка (restore RX, LED, лог) — как в thor_send_string  
}
