// ============================================================  
// jtty_modem.cpp — передатчик JTTY (WSJT-X 3.3 совместимый)  
//  
// Протокол:  
//   Кадр 1.888 с = 59 символов 4-FSK, 31.25 бод (32 мс/символ):  
//     13 тонов sync (is13) + 46 тонов данных (92 кодированных бита).  
//   Payload: 32 бита атом + reserved(0) + EOM = 34 бита.  
//   FEC: CRC-12 (poly 0x80F) -> 46 бит -> tail-biting conv K=10  
//        r=1/2, генераторы 0o1167/0o1545 -> 92 бита -> 46 дибитов.  
//   Тоны: f0 + tone*31.25 Гц, f0 = FREQ_JTTY - 46.875 Гц (центрирование).  
//  
// Эталон: lib/jtty/*.f90, packjt77.f90, tbcc.f90, nhash.c (WSJT-X).  
// ============================================================  
  
#include <Arduino.h>  
#include <stdint.h>  
#include <stdbool.h>  
#include <string.h>  
#include <ctype.h>  
#include "jtty_modem.h"  
#include "thor_modem.h"     // mfsk_prepare, mfsk_emit_tone, mfsk_carrier, mfsk_sync_deadline, mfsk_wait_symbol  
#include "vfo_hardware.h"   // vfo_set_tone_instant, device_SI  
  
// ---------- Константы протокола ----------  
#define JTTY_SYMBOL_US      32000UL            // 32 мс  
#define JTTY_BAUD_HZ        31.25  
#define JTTY_NTONES         4  
#define JTTY_NSYNC          13  
#define JTTY_NDATA          46  
#define JTTY_NSYMS          59                 // 13 + 46  
#define JTTY_MAX_FRAMES     16  
#define JTTY_CENTER_OFFSET  (1.5 * JTTY_BAUD_HZ)  // 46.875 Гц  
  
static const uint8_t JTTY_IS13[JTTY_NSYNC] =  
    {0,2,2,3,0,0,3,2,1,3,1,2,0};  
  
// Алфавит источника (jtty_source_codec.f90), 64 символа, индексы 0..63  
static const char JTTY_ALPHABET[] =  
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?!\"#$%,&*()_'=[]{}<>|:;";  
  
// ---------- Типы ----------  
typedef struct {  
    int kind;       // JTTY_ATOM_*  
    int subtype;    // действие/подтип  
    int role;       // JTTY_ROLE_*  
    int value;  
    int value2;  
    char text[14];  
} JttyAtom;  
  
typedef struct {  
    uint64_t frames[JTTY_MAX_FRAMES];   // 34-битные payload'ы  
    int nframes;  
} JttyFrameList;  
  
enum {  
    JTTY_ATOM_CALL = 0, JTTY_ATOM_EXCH_NUM = 1, JTTY_ATOM_EXCH_LOC = 2,  
    JTTY_ATOM_EXCH_PAIR = 3, JTTY_ATOM_EXCH_NUM_TIME = 4,  
    JTTY_ATOM_CONTROL = 5, JTTY_ATOM_GRID4 = 6, JTTY_ATOM_TEXT5 = 7  
};  
enum { JTTY_CALL_CQ=0, JTTY_CALL_CALL=1, JTTY_CALL_TU_CQ=2,  
       JTTY_CALL_CALL_TU=3, JTTY_CALL_CALL_AGN=4, JTTY_CALL_TU_NOW=5 };  
enum { JTTY_ROLE_FIELD_ONLY=0, JTTY_ROLE_FULL=1 };  
enum { JTTY_EXCHANGE_UNKNOWN=0, JTTY_EXCHANGE_FIELD_DAY=1, JTTY_EXCHANGE_RTTY=2 };  

// CONTROL-подтипы (jtty_source_codec.f90)  
enum { JTTY_CONTROL_AGN=0, JTTY_CONTROL_CALL=1, JTTY_CONTROL_AGN_CALL=2,  
       JTTY_CONTROL_NR=3,  JTTY_CONTROL_AGN_NR=4, JTTY_CONTROL_EXCH=5,  
       JTTY_CONTROL_STATE=6, JTTY_CONTROL_SECTION=7, JTTY_CONTROL_ZONE=8,  
       JTTY_CONTROL_GRID=9,  JTTY_CONTROL_RPRT=10, JTTY_CONTROL_QSL_TU=11,  
       JTTY_CONTROL_TU=12,   JTTY_CONTROL_QRZ=13, JTTY_CONTROL_QSO_B4=14,  
       JTTY_CONTROL_WAIT=15, JTTY_CONTROL_NIL=16,  JTTY_CONTROL_OK=17 };

uint32_t JTTY_Base_freq = 10001500;  
  
// ============================================================  
// Алфавит / утилиты  
// ============================================================  
static int source_index(char c) {  
    const char* p = strchr(JTTY_ALPHABET, c);  
    return p ? (int)(p - JTTY_ALPHABET) : -1;  
}  
  
static char source_char(int i) {  
    return (i >= 0 && i < 64) ? JTTY_ALPHABET[i] : ' ';  
}  
  
// ============================================================  
// nhash — Jenkins hashlittle (байтовый путь, эталон nhash.c)  
// Используется только для токенов CQ_aaaa.  
// ============================================================  
static inline uint32_t rot32(uint32_t x, int k) { return (x << k) | (x >> (32 - k)); }

static inline void nh_mix(uint32_t& a, uint32_t& b, uint32_t& c) {  
    a-=c; a^=rot32(c,4);  c+=b;  
    b-=a; b^=rot32(a,6);  a+=c;  
    c-=b; c^=rot32(b,8);  b+=a;  
    a-=c; a^=rot32(c,16); c+=b;  
    b-=a; b^=rot32(a,19); a+=c;  
    c-=b; c^=rot32(b,4);  b+=a;
}  

static inline void nh_final(uint32_t& a, uint32_t& b, uint32_t& c) {  
    c^=b; c-=rot32(b,14);  
    a^=c; a-=rot32(c,11);  
    b^=a; b-=rot32(a,25);  
    c^=b; c-=rot32(b,16);  
    a^=c; a-=rot32(c,4);  
    b^=a; b-=rot32(a,14);  
    c^=b; c-=rot32(b,24);
}
  
static int32_t nhash_bytes(const void* key, size_t length, int32_t initval) {  
    const uint8_t* k = (const uint8_t*)key;  
    uint32_t a = 0xdeadbeef + (uint32_t)length + (uint32_t)initval, b = a, c = a;  
    while (length > 12) {  
        a += k[0] | ((uint32_t)k[1]<<8) | ((uint32_t)k[2]<<16) | ((uint32_t)k[3]<<24);  
        b += k[4] | ((uint32_t)k[5]<<8) | ((uint32_t)k[6]<<16) | ((uint32_t)k[7]<<24);  
        c += k[8] | ((uint32_t)k[9]<<8) | ((uint32_t)k[10]<<16)| ((uint32_t)k[11]<<24);  
        nh_mix(a,b,c); k += 12; length -= 12;  
    }  
    switch (length) {  
        case 12: c += (uint32_t)k[11]<<24; /* fall through */  
        case 11: c += (uint32_t)k[10]<<16; /* fall through */  
        case 10: c += (uint32_t)k[9]<<8;   /* fall through */  
        case 9:  c += k[8];                /* fall through */  
        case 8:  b += (uint32_t)k[7]<<24;  /* fall through */  
        case 7:  b += (uint32_t)k[6]<<16;  /* fall through */  
        case 6:  b += (uint32_t)k[5]<<8;   /* fall through */  
        case 5:  b += k[4];                /* fall through */  
        case 4:  a += (uint32_t)k[3]<<24;  /* fall through */  
        case 3:  a += (uint32_t)k[2]<<16;  /* fall through */  
        case 2:  a += (uint32_t)k[1]<<8;   /* fall through */  
        case 1:  a += k[0]; break;
        case 0:  return (int32_t)(c & 32767);  
    }  
    nh_final(a,b,c);  
    return (int32_t)(c & 32767);  
}  
  
// hash() из packjt77.f90: nhash(s, len, 146) & 32767  
static int jtty_hash15(const char* s, size_t len) {  
    return (int)nhash_bytes(s, len, 146);  
}  
  
// ============================================================  
// ihashcall — 22/12/10-битный хэш позывного  
// n8 = base-38 по первым 11 символам; хэш = верхние m бит  
// младших 64 бит произведения 47055833459 * n8.  
// ============================================================  
static int ihashcall(const char* c0_13, int m) {  
    static const char C38[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";  
    uint64_t n8 = 0;  
    char buf[12];  
    memset(buf, ' ', 11); buf[11] = 0;  
    size_t l = strlen(c0_13); if (l > 11) l = 11;  
    memcpy(buf, c0_13, l);  
    for (int i = 0; i < 11; i++) {  
        const char* p = strchr(C38, buf[i]);  
        n8 = 38 * n8 + (uint64_t)(p ? (p - C38) : 0);  
    }  
    uint64_t prod = 47055833459ULL * n8;      // переполнение = low 64 бита  
    if (m <= 0) return 0;  
    return (int)(prod >> (64 - m));  
}  
  
// pack28 — точный порт pack28() из packjt77.f90 (WSJT-X 3.3).  
// Возвращает 28-битное значение или -1 (невалидный вход).  

// Константы pack28 (из packjt77.f90: NTOKENS=2063592, MAX22=4194304)   
#define PACK28_NTOKENS  2063592L  
#define PACK28_MAX22    4194304L  
  
// Алфавиты индексации стандартного позывного  
static const char P28_A1[38] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; // 37  
static const char P28_A2[37] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";  // 36  
static const char P28_A3[11] = "0123456789";                            // 10  
static const char P28_A4[28] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";           // 27  

static int idx_in(const char* tab, char c)  
{  
    const char* p = strchr(tab, c);  
    return p ? (int)(p - tab) : -1;  
}  
  
static bool is_dig(char c)  { return c >= '0' && c <= '9'; }  
static bool is_let(char c)  { return c >= 'A' && c <= 'Z'; }  
  
// jtty_modem.cpp — порт pack28 из packjt77.f90.  
// Семантика: спец-токены (0..NTOKENS-1), хэши <...> (NTOKENS..NTOKENS+MAX22-1),  
// стандартные позывные (NTOKENS+MAX22 и выше). Возврат: n28 ≥ 0, -1 при ошибке.  
  
int32_t pack28(const char* call)  
{  
    // --- c13: строка до 13 символов, дополнена пробелами справа (adjustl) ---  
    char c13[14];  
    memset(c13, ' ', 13);  
    c13[13] = 0;  
    size_t L = strlen(call);  
    if (L > 13) L = 13;  
    memcpy(c13, call, L);  
  
    // --- Workaround'ы префиксов — ДО токенов и iarea (эталон делает так) ---  
    // 3DA0xxx → 3D0xxx : удалить 'A' на позиции 2, подтянуть хвост  
    if (!strncmp(c13, "3DA0", 4)) {  
        c13[2] = '0';  
        memmove(c13 + 3, c13 + 4, 9);   // "3DA0ABC     " → "3D0ABC      "  
    }  
    // 3Xyyy(буква) → Qyyy  
    if (c13[0] == '3' && c13[1] == 'X' && is_let(c13[2])) {  
        c13[0] = 'Q';  
    }  
  
    // n = длина trim(c13) — без хвостовых пробелов  
    int n = 13;  
    while (n > 0 && c13[n - 1] == ' ') n--;  
  
    // --- Спец-токены ---  
    if (n >= 2 && !strncmp(c13, "DE", 2) && (n == 2 || c13[2] == ' '))  
        return 0;  
    if (n >= 3 && !strncmp(c13, "QRZ", 3) && (n == 3 || c13[3] == ' '))  
        return 1;  
    if (n >= 2 && !strncmp(c13, "CQ", 2) && (n == 2 || c13[2] == ' '))  
        return 2;  
  
    // CQ_nnn: «CQ» + пробел + 1..3 цифры → n28 = 3 + nnn  
    if (n >= 4 && !strncmp(c13, "CQ ", 3)) {  
        bool digits = true;  
        int nd = 0, val = 0;  
        for (int i = 3; i < n; i++) {  
            if (!is_dig(c13[i])) { digits = false; break; }  
            val = val * 10 + (c13[i] - '0');  
            nd++;  
        }  
        if (digits && nd >= 1 && nd <= 3)  
            return (3 + val) & 0x0FFFFFFFL;  
  
        // CQ_aaaa: 4 буквы → n28 = 3 + 1000 + jtty_hash15(c13+4,4)  
        if (n == 8) {  
            bool lets = true;  
            for (int i = 4; i < 8; i++)  
                if (!is_let(c13[i])) { lets = false; break; }  
            if (lets)  
                return (3 + 1000 + jtty_hash15(c13 + 4, 4)) & 0x0FFFFFFFL;  
        }  
    }  
    // QRZ_aaaa — тот же диапазон буквенных токенов, что и CQ_aaaa  
    if (n == 8 && !strncmp(c13, "QRZ ", 4)) {  
        bool lets = true;  
        for (int i = 4; i < 8; i++)  
            if (!is_let(c13[i])) { lets = false; break; }  
        if (lets)  
            return (3 + 1000 + jtty_hash15(c13 + 4, 4)) & 0x0FFFFFFFL;  
    }  
  
    // --- Стандартный позывной: iarea = позиция первой цифры при обходе справа ---  
    // Fortran: 1-based; здесь 0-based + 1 → iarea1 ∈ {2,3}  
    int iarea1 = 0;  
    for (int i = n - 1; i >= 0; i--) {  
        if (is_dig(c13[i])) { iarea1 = i + 1; break; }  
    }  
    if (iarea1 != 2 && iarea1 != 3) {  
        // Нет цифры или цифра на позиции 0 / ≥4 → хэш-ветка (<...>-позывной)  
        int32_t n22 = ihashcall(c13, 22);  
        return (PACK28_NTOKENS + n22) & 0x0FFFFFFFL;  
    }  
  
    // Позывной не длиннее 6 символов  
    if (n > 6) {  
        int32_t n22 = ihashcall(c13, 22);  
        return (PACK28_NTOKENS + n22) & 0x0FFFFFFFL;  
    }  
  
    // После цифры допустим максимум 2 буквы (суффикс типа /P исключён выше по n>6)  
    // Эталон: цифра + до 2-х символов в поле i4..i6 по алфавиту a4 — проверяется  
    // idx_in ниже; явной валидации формы здесь не требуется.  
  
    // --- callsign из 6 символов ---  
    char cs[7];  
    memset(cs, ' ', 6);  
    cs[6] = 0;  
    if (iarea1 == 2) {  
        // iarea=2 → callsign = ' ' + c13(1:5)  (зона на 2-й позиции)  
        cs[0] = ' ';  
        memcpy(cs + 1, c13, 5);  
    } else {  
        // iarea=3 → callsign = c13(1:6)  (зона на 3-й позиции)  
        memcpy(cs, c13, 6);  
    }  
  
    int i1 = idx_in(P28_A1, cs[0]);   // ' '+цифра+буква   (37)  
    int i2 = idx_in(P28_A2, cs[1]);   // цифра+буква       (36)  
    int i3 = idx_in(P28_A3, cs[2]);   // цифра             (10)  
    int i4 = idx_in(P28_A4, cs[3]);   // ' '+буква         (27)  
    int i5 = idx_in(P28_A4, cs[4]);  
    int i6 = idx_in(P28_A4, cs[5]);  
  
    if (i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 || i5 < 0 || i6 < 0) {  
        // Форма не легла в стандартную сетку → хэш  
        int32_t n22 = ihashcall(c13, 22);  
        return (PACK28_NTOKENS + n22) & 0x0FFFFFFFL;  
    }  
  
    int64_t n28 = 36LL*10*27*27*27*i1 + 10LL*27*27*27*i2 + 27LL*27*27*i3  
                + 27LL*27*i4 + 27LL*i5 + i6;  
    n28 += (int64_t)PACK28_NTOKENS + PACK28_MAX22;  
    return (int32_t)(n28 & 0x0FFFFFFFL);  
}
  
// ============================================================  
// Атомы -> 32-битное тело  
// ============================================================  
static inline uint32_t finish_struct(uint32_t body, int family) {  
    return 4u * (8u * body + (uint32_t)family) + 2u;  
}  
  
// GRID4 (AA00..RR99) -> index (a*18+b)*10+c)*10+d  
static bool grid4_to_index(const char* g, int* idx) {  
    if (!g[0] || !g[1] || !g[2] || !g[3]) return false;  
    if (g[0] < 'A' || g[0] > 'R' || g[1] < 'A' || g[1] > 'R') return false;  
    if (!isdigit((unsigned char)g[2]) || !isdigit((unsigned char)g[3])) return false;  
    *idx = ((g[0]-'A')*18 + (g[1]-'A'))*100 + (g[2]-'0')*10 + (g[3]-'0');  
    return true;  
}


static bool pos_incomplete(int len, int na);

static bool standard_call(const char* call) {  
    return pack28(call) > (int32_t)(PACK28_NTOKENS + PACK28_MAX22);
}  

// pack_jtty_atom: атом + eom -> 34-битный payload (биты: n32<<2 | eom)  
static bool jtty_pack_atom(const JttyAtom* atom, bool eom, uint64_t* payload34) {  
    uint32_t n32 = 0;  
    switch (atom->kind) {  
    case JTTY_ATOM_CALL: {  
        if (atom->subtype < 0 || atom->subtype > 5) return false;  
        if (!standard_call(atom->text)) return false;  
        int32_t n28 = pack28(atom->text);
        if (n28 < 0) return false;  
        int i2, n2;  
        if (atom->subtype <= 3) { i2 = 0; n2 = atom->subtype; }  
        else                    { i2 = 1; n2 = atom->subtype - 4; }  
        n32 = ((uint32_t)n28 << 4) | ((uint32_t)n2 << 2) | (uint32_t)i2;  
        break;  
    }  
    case JTTY_ATOM_EXCH_NUM: {  
        if (atom->role < 0 || atom->role > 1) return false;  
        if (atom->subtype < 0 || atom->subtype > 15) return false;  
        if (atom->value < 0 || atom->value > 131071) return false;  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)atom->value << 5);  
        n32 = finish_struct(body, 0);  
        break;  
    }  
    case JTTY_ATOM_EXCH_LOC: {  
        if (atom->role < 0 || atom->role > 1) return false;  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x1FFFF) << 5);  
        n32 = finish_struct(body, 1);  
        break;  
    }  
    case JTTY_ATOM_EXCH_PAIR: {  
        uint32_t body = ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x3FF) << 12)  
                      | ((uint32_t)(atom->value2 & 0x3FF) << 2);  
        n32 = finish_struct(body, 2);       // см. заметку: раскладка pair_data  
        break;                              // сверить с case в jtty_source_codec.f90  
    }  
    case JTTY_ATOM_EXCH_NUM_TIME: {  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x1FFFF) << 5);  
        n32 = finish_struct(body, 3);  
        break;  
    }  
    case JTTY_ATOM_CONTROL: {  
        if (atom->subtype < 0 || atom->subtype > 17) return false;  
        uint32_t body = (0u << 23) | ((uint32_t)atom->subtype << 16);  // JTTY_MISC_CONTROL=0  
        n32 = finish_struct(body, 4);  
        break;  
    }  
    case JTTY_ATOM_GRID4: {  
        if (atom->role < 0 || atom->role > 1) return false;  
        int gi;  
        if (!grid4_to_index(atom->text, &gi)) return false;  
        uint32_t data = ((uint32_t)atom->role << 22) | ((uint32_t)gi << 7);  
        uint32_t body = (1u << 23) | data;                              // JTTY_MISC_GRID4=1  
        n32 = finish_struct(body, 4);  
        break;  
    }  
    case JTTY_ATOM_TEXT5: {  
        uint32_t top30 = 0;  
        for (int i = 0; i < 5; i++) {  
            char c = atom->text[i] ? atom->text[i] : ' ';  
            int ix = source_index(c);  
            if (ix < 0) return false;  
            top30 = 64u * top30 + (uint32_t)ix;  
        }  
        n32 = 4u * top30 + 3u;  
        break;  
    }  
    default:  
        return false;  
    }  
    // frame(1:32)=n32 MSB-first, frame(33)='0' reserved, frame(34)=eom  
    *payload34 = ((uint64_t)n32 << 2) | (eom ? 1ULL : 0ULL);  
    return true;  
}  
  
static bool jtty_pack_atoms(const JttyAtom* atoms, int natoms,  
                            uint64_t* frames, int* nframes, int is_final) {  
    bool final_ok = (is_final < 0) || (is_final != 0);  
    *nframes = 0;  
    if (natoms < 1 || natoms > JTTY_MAX_FRAMES) return false;  
    for (int i = 0; i < natoms; i++) {  
        bool eom = (i == natoms - 1) && final_ok;  
        if (!jtty_pack_atom(&atoms[i], eom, &frames[i])) return false;  
    }  
    *nframes = natoms;  
    return true;  
}  
  
// ============================================================  
// Нормализация и минимальный пакер сообщения  
// Полный DP-паковщик (pack_jtty) с contest-грамматикой — отдельный  
// слой; для маяка достаточно: CALL-атом + TEXT5-атомы по 5 символов.  
// ============================================================  
static void normalize_jtty(char* s) {  
    // верхний регистр, '~'->' ', недопустимые -> '#', схлопывание пробелов  
    for (char* p = s; *p; p++) {  
        *p = (char)toupper((unsigned char)*p);  
        if (*p == '~') *p = ' ';  
        if (source_index(*p) < 0) *p = '#';  
    }  
    // схлопывание пробелов + trim  
    char* r = s; char* w = s; bool sp = true;  
    while (*r) {  
        if (*r == ' ') { if (!sp) *w++ = ' '; sp = true; }  
        else { *w++ = *r; sp = false; }  
        r++;  
    }  
    if (w > s && w[-1] == ' ') w--;  
    *w = 0;  
}  
  
static bool is_standard_callsign(const char* s) {  
    size_t n = strlen(s);  
    if (n < 3 || n > 6) return false;  
    if (strchr(s, '/') || strchr(s, '<')) return false;  
    int nd = 0;  
    for (size_t i = 0; s[i]; i++) if (isdigit((unsigned char)s[i])) nd++;  
    return nd >= 1;   // упрощённо; формула pack28 сама отсеет невалидное  
}  
  
// Сообщение -> кадры. Правила маяка:  
//   "CQ <call> CQ" / "CQ <call>"  -> CALL-атом subtype CQ  
//   "<call>"                     -> CALL-атом subtype CALL  
//   "<call> TU"                  -> CALL_TU;  "TU <call> CQ" -> TU_CQ  
//   остальной текст              -> TEXT5-атомы по 5 символов (без пробелов —  
//   пробел кодируем как индекс алфавита 36, он в ALPHABET есть).  
static bool jtty_pack_message(const char* umsg_in, JttyFrameList* fl,  
                              int exchange_profile, int is_final) {  
    (void)exchange_profile;  
    char msg[81]; strncpy(msg, umsg_in, 80); msg[80] = 0;  
    normalize_jtty(msg);  
    if (!msg[0]) return false;  
  
    JttyAtom atoms[JTTY_MAX_FRAMES];  
    int na = 0;  
  
    // Разбор по словам  
    char buf[81]; strcpy(buf, msg);  
    char* w[8]; int nw = 0;  
    for (char* t = strtok(buf, " "); t && nw < 8; t = strtok(NULL, " ")) w[nw++] = t;  
  
    if (nw >= 2 && strcmp(w[0], "CQ") == 0 && is_standard_callsign(w[1])) {  
        atoms[na].kind = JTTY_ATOM_CALL; atoms[na].subtype = JTTY_CALL_CQ;  
        atoms[na].role = JTTY_ROLE_FIELD_ONLY;  
        strncpy(atoms[na].text, w[1], 13); atoms[na].text[13] = 0;  
        na++;
        // съедено "CQ <call>" (+ возможный хвост "CQ" игнорируем)  
        // хвост сообщения после второго слова уходит в TEXT5 ниже  
        int consumed = 3 + (int)strlen(w[1]);          // "CQ " + call  
        char* rest = msg + consumed; while (*rest == ' ') rest++;  
        if (strncmp(rest, "CQ", 2) == 0 && (rest[2] == 0 || rest[2] == ' ')) {  
            rest += 2; while (*rest == ' ') rest++;  
        }  
        memmove(msg, rest, strlen(rest) + 1);  
    } else if (nw >= 1 && is_standard_callsign(w[0]) && nw <= 2) {  
        int subtype = JTTY_CALL_CALL;  
        if (nw == 2 && strcmp(w[1], "TU") == 0) subtype = JTTY_CALL_CALL_TU;  
        atoms[na].kind = JTTY_ATOM_CALL; atoms[na].subtype = subtype;  
        atoms[na].role = JTTY_ROLE_FIELD_ONLY;  
        strncpy(atoms[na].text, w[0], 13); atoms[na].text[13] = 0;  
        na++;
        msg[0] = 0;  
    }  
  
    // Остаток сообщения — TEXT5-атомами по 5 символов  
    int len = (int)strlen(msg);  
    for (int pos = 0; pos < len && na < JTTY_MAX_FRAMES; pos += 5) {  
        JttyAtom* a = &atoms[na];  
        memset(a, 0, sizeof(*a));  
        a->kind = JTTY_ATOM_TEXT5; a->role = JTTY_ROLE_FIELD_ONLY;  
        int k = 0;  
        for (int i = pos; i < pos + 5 && i < len; i++) a->text[k++] = msg[i];  
        while (k < 5) a->text[k++] = ' ';  
        a->text[5] = 0;  
        na++;  
    }  
    if (na == 0) return false;  
    if (len > 0 && na >= JTTY_MAX_FRAMES && pos_incomplete(len, na)) return false;  
  
    return jtty_pack_atoms(atoms, na, fl->frames, &fl->nframes, is_final);  
}  
static bool pos_incomplete(int len, int na) { return len > na * 5; }  
  
// ============================================================  
// FEC: CRC-12(0x80F) + tail-biting conv K=10 (0o1167/0o1545)  
// ============================================================  
#define JTTY_PAYLOAD_BITS 34  
#define JTTY_INFO_BITS    46          // 34 + 12  
#define JTTY_NU           9  
#define JTTY_G0           01167       // octal  
#define JTTY_G1           01545  
#define JTTY_CRC_POLY     0x80F  
  
static inline int parity32(uint32_t x) {  
    x ^= x >> 16; x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;  
    return x & 1;  
}  
  
// info_bits[0..33] = payload MSB-first. Возвращает 12 бит CRC.  
static uint16_t jtty_crc12(const uint8_t* info_bits /*34*/) {  
    uint16_t reg = 0;  
    for (int i = 0; i < JTTY_PAYLOAD_BITS; i++) {  
        uint16_t fb = ((reg >> 11) ^ info_bits[i]) & 1;  
        reg = (uint16_t)(((reg << 1) & 0xFFF) ^ (fb ? JTTY_CRC_POLY : 0));  
    }  
    return reg;  
}  
  
// payload34 -> itone[59]: is13 + 46 дибитов  
static void jtty_build_frame(uint64_t payload34, uint8_t* itone /*59*/) {  
    uint8_t info[JTTY_INFO_BITS];  
    for (int i = 0; i < JTTY_PAYLOAD_BITS; i++)  
        info[i] = (uint8_t)((payload34 >> (33 - i)) & 1);  
  
    uint16_t crc = jtty_crc12(info);  
    for (int i = 0; i < 12; i++)  
        info[JTTY_PAYLOAD_BITS + i] = (uint8_t)((crc >> (11 - i)) & 1);  
  
    // Tail-biting: регистр = последние NU бит информационного слова  
    uint16_t reg10 = 0;  
    for (int t = 0; t < JTTY_NU; t++)  
        reg10 = (uint16_t)((reg10 << 1) | info[JTTY_INFO_BITS - JTTY_NU + t]);  
  
    for (int i = 0; i < JTTY_NSYNC; i++) itone[i] = JTTY_IS13[i];  // sync-преамбула

    for (int i = 0; i < JTTY_INFO_BITS; i++) {  
        reg10 = (uint16_t)(((reg10 << 1) | info[i]) & 0x3FF);  
        uint8_t b0 = (uint8_t)parity32(reg10 & JTTY_G0);  
        uint8_t b1 = (uint8_t)parity32(reg10 & JTTY_G1);  
        itone[JTTY_NSYNC + i] = (uint8_t)((b0 << 1) | b1);  
        // ПОРЯДОК ДИБИТА: если WSJT-X рисует кадр зеркально по частоте,  
        // заменить на (b1 << 1) | b0 — сверить с tbcc.f90 укладкой пары.  
    }  
}  
  
// ============================================================  
// TX  
// ============================================================  
// Готовит тоновую сетку: base = нижний тон, шаг 31.25 Гц, 4 тона.  
// freq_hz — центр полосы (FREQ_JTTY).  
void prepare_jtty_frequencies(uint32_t freq_hz) {  
    JTTY_Base_freq = freq_hz;  
    double f0 = (double)freq_hz - JTTY_CENTER_OFFSET;   // центрирование  
    mfsk_prepare((uint32_t)(f0 + 0.5), JTTY_BAUD_HZ, JTTY_NTONES);  
}  
  
// Один кадр: 59 символов по 32 мс.  
static bool jtty_tx_frame(const uint8_t* itone) {  
    for (int i = 0; i < JTTY_NSYMS; i++) {  
        if (!mfsk_emit_tone(itone[i], JTTY_SYMBOL_US)) return false;  
    }  
    return true;  
}  
  
// Передача сообщения (блокирующая, как остальные модемы).  
void jtty_send_string(const char* str) {  
    if (!str || !*str) return;  
    if (!device_SI[0]) {  
        // PIO-ветка: сетка уже подготовлена prepare_jtty_frequencies  
    }  
    mfsk_tx_begin();  
  
    JttyFrameList fl;  
    if (!jtty_pack_message(str, &fl, JTTY_EXCHANGE_UNKNOWN, -1)) {  
        Serial.println(F("[JTTY] упаковка сообщения не удалась"));  
        return;  
    }  
  
    mfsk_carrier(false);  
    mfsk_sync_deadline();  
  
    mfsk_carrier(true);  
    for (int f = 0; f < fl.nframes; f++) {  
        uint8_t itone[JTTY_NSYMS];  
        jtty_build_frame(fl.frames[f], itone);  
        if (!jtty_tx_frame(itone)) { mfsk_carrier(false); return; }  
    }  
    mfsk_carrier(false);  
  
    Serial.printf("[JTTY] передано кадров: %d\n", fl.nframes);  
}



// ============================================================  
// jtty_selftest — проверка кодека по эталонным векторам WSJT-X.  
// Вывод в Serial. Вызов: команда "test jtty" в консоли.  
// Вектора хэшей — из test_packjt77_hash_vectors.f90 (ihashcall m=10/12/22).  
// ============================================================  
void jtty_selftest(void)  
{  
    Serial.println(F("\n[JTTY TEST] start"));  
  
    // --- Тест 1: ihashcall по 13 эталонным векторам ---  
    struct { const char* call; int h10, h12, h22; } hv[] = {  
        { "K1ABC",       712, 2851, 2920267 },  
        { "W9XYZ",       972, 3889, 3982604 },  
        { "PJ4/K1ABC",   346, 1387, 1420834 },  
        { "KH1/KH7Z",    201,  806,  825805 },  
        { "K1JT/P",      959, 3839, 3931158 },  
        { "W3CCX",       665, 2662, 2726483 },  
        { "3DA0ABC",      48,  192,  196695 },  
        { "3XABC",       563, 2254, 2308721 },  
        { "///////////", 671, 2685, 2749801 },  
        { "ZZZZZZZZZZZ", 902, 3609, 3695718 },  
        { "99999999999", 762, 3050, 3123740 },  
        { "           ",   0,    0,       0 },  
    };  
    int pass = 0, total = 0;  
    Serial.println(F("--- ihashcall ---"));  
    for (unsigned i = 0; i < sizeof(hv)/sizeof(hv[0]); i++) {  
        int g10 = ihashcall(hv[i].call, 10);  
        int g12 = ihashcall(hv[i].call, 12);  
        int g22 = ihashcall(hv[i].call, 22);  
        bool ok = (g10 == hv[i].h10 && g12 == hv[i].h12 && g22 == hv[i].h22);  
        total++; pass += ok;  
        Serial.printf("  %-12s 10:%4d 12:%4d 22:%7d  %s\n",  
                      hv[i].call, g10, g12, g22, ok ? "OK" : "FAIL");  
    }  
  
    // --- Тест 2: pack28 токены и стандартные позывные ---  
    Serial.println(F("--- pack28 ---"));  
    struct { const char* s; int32_t lo, hi; } pv[] = {  
        { "DE",        0, 0 }, { "QRZ", 1, 1 }, { "CQ", 2, 2 },  
        { "K1ABC",     PACK28_NTOKENS + PACK28_MAX22, 0x0FFFFFFFL }, // стандартная ветка  
        { "RU0AOG",    PACK28_NTOKENS + PACK28_MAX22, 0x0FFFFFFFL },  
        { "PJ4/K1ABC", PACK28_NTOKENS + 1420834, PACK28_NTOKENS + 1420834 }, // хэш  
    };  
    for (unsigned i = 0; i < sizeof(pv)/sizeof(pv[0]); i++) {  
        int32_t n28 = pack28(pv[i].s);  
        bool ok = (n28 >= pv[i].lo && n28 <= pv[i].hi);  
        total++; pass += ok;  
        Serial.printf("  pack28(%-10s) = %9ld  %s\n", pv[i].s, (long)n28, ok ? "OK" : "FAIL");  
    }  
  
    // --- Тест 3: атом TEXT5 'HELLO' -> эталон 0x11395558D (34 бита, eom=1) ---  
    Serial.println(F("--- pack atom ---"));  
    {  
        JttyAtom a; memset(&a, 0, sizeof a);  
        a.kind = JTTY_ATOM_TEXT5; strncpy(a.text, "HELLO", sizeof(a.text) - 1);  
        uint64_t p34 = 0;  
        bool ok = jtty_pack_atom(&a, true, &p34) && (p34 == 0x11395558DULL);  
        total++; pass += ok;  
        Serial.printf("  TEXT5 'HELLO' payload=0x%09llX  %s (expect 0x11395558D)\n",  
                      (unsigned long long)p34, ok ? "OK" : "FAIL");  
    }  
    // CONTROL AGN? -> эталон 0x000000049 (eom=1)  
    {  
        JttyAtom a; memset(&a, 0, sizeof a);  
        a.kind = JTTY_ATOM_CONTROL; a.subtype = JTTY_CONTROL_AGN;  
        uint64_t p34 = 0;  
        bool ok = jtty_pack_atom(&a, true, &p34) && (p34 == 0x000000049ULL);  
        total++; pass += ok;  
        Serial.printf("  CONTROL 'AGN?' payload=0x%09llX  %s (expect 0x000000049)\n",  
                      (unsigned long long)p34, ok ? "OK" : "FAIL");  
    }  
  
    // --- Тест 4: полный кадр — дамп 59 тонов ---  
    Serial.println(F("--- frame ---"));  
    {  
        uint8_t itone[JTTY_NSYMS];  
        jtty_build_frame(0x11395558DULL, itone);  
        // первые 13 должны быть is13  
        bool ok = true;  
        for (int i = 0; i < JTTY_NSYNC; i++) if (itone[i] != JTTY_IS13[i]) ok = false;  
        total++; pass += ok;  
        Serial.print(F("  sync+data: "));  
        for (int i = 0; i < JTTY_NSYMS; i++) Serial.printf("%d", itone[i]);  
        Serial.printf("  sync:%s\n", ok ? "OK" : "FAIL");  
    }  
  
    Serial.printf("[JTTY TEST] %d/%d passed\n\n", pass, total);  
}





// Мини-фреймворк: expect_pack(text, want_nf, i2a, n2a, i2b, n2b)  
// -1 = не проверять. Счётчик pass/fail как в jtty_selftest.  
  
static int tp_pass, tp_fail;  
  
static void expect_pack(const char* text, int want_nf,  
                        int i2a, int n2a, int i2b, int n2b, int i2c = -1)

{  
    char msg[81];  
    strncpy(msg, text, 80); msg[80] = 0;  
    JttyFrameList fl;  
    bool ok = jtty_pack_message(text, &fl, JTTY_EXCHANGE_UNKNOWN, -1);
    tp_pass++;  
    if (!ok) { Serial.printf("  FAIL pack \"%s\"\n", text); tp_fail++; tp_pass--; return; }  
    if (fl.nframes != want_nf) {  
        Serial.printf("  FAIL nf \"%s\": want %d got %d\n", text, want_nf, fl.nframes);  
        tp_fail++; tp_pass--; return;  
    }  
    // биты: i2 = n32&3, n2 = (n32>>2)&3; n32 = payload>>2  
    auto low4 = [](uint64_t pl, int& i2, int& n2){  
        uint32_t n32 = (uint32_t)(pl >> 2);  
        i2 = n32 & 3; n2 = (n32 >> 2) & 3;  
    };  
    // reserved bit 33 и EOM  
    for (int i = 0; i < fl.nframes; i++) {  
        if (fl.frames[i] & 2ULL) { Serial.printf("  FAIL reserved bit \"%s\" f%d\n", text, i); tp_fail++; tp_pass--; }  
        bool eom = fl.frames[i] & 1ULL;  
        if (eom != (i == fl.nframes - 1)) { Serial.printf("  FAIL eom \"%s\" f%d\n", text, i); tp_fail++; tp_pass--; }  
    }  
    if (want_nf >= 1) { int a,b; low4(fl.frames[0], a, b);  
        if ((i2a >= 0 && a != i2a) || (n2a >= 0 && b != n2a)) {
            Serial.printf("  FAIL f1 \"%s\": want %d.%d got %d.%d\n", text, i2a, n2a, a, b);  
            tp_fail++; tp_pass--; } }  
    if (want_nf >= 2 && i2b >= 0) { int a,b; low4(fl.frames[1], a, b);  
        if (a != i2b || (n2b >= 0 && b != n2b)) {  
            Serial.printf("  FAIL f2 \"%s\": want %d.%d got %d.%d\n", text, i2b, n2b, a, b);  
            tp_fail++; tp_pass--; } }  
}  
  
void jtty_selftest_pack()  
{  
    tp_pass = tp_fail = 0;  
    // основная таблица из test_jtty_pack (без профиля)  
    expect_pack("CQ KA1ABC CQ",            1, 0, 0, -1, -1);  
    expect_pack("CQ K1ABC CQ",             1, 0, 0, -1, -1);  
    expect_pack("K1A",                     1, 0, 1, -1, -1);   // bare call? — сверить: 'K1A' нестандартный → TEXT5? Эталон говорит 0.1  
    expect_pack("WB9XYZ",                  1, 0, 1, -1, -1);  
    expect_pack("TU WB9XYZ CQ",            1, 0, 2, -1, -1);  
    expect_pack("WB9XYZ TU",               1, 0, 3, -1, -1);  
    expect_pack("WB9XYZ AGN?",             1, 1, 0, -1, -1);  
    expect_pack("TU NOW WB9XYZ",           1, 1, 1, -1, -1);  
    expect_pack("WB9XYZ TU CQ KA1ABC CQ",  2, 0, 3,  0,  0);  
    expect_pack("K1ABC TU NOW W1XYZ",      2, 0, 1,  1,  1);  
    expect_pack("WB9XYZ 599 123",          2, 0, 1,  2, -1);  
    expect_pack("599 MA",                  1, 2, -1, -1, -1);  
    expect_pack("599 FN42",                1, 2, -1, -1, -1);  
    expect_pack("FN42",                    1, 2, -1, -1, -1);  
    expect_pack("1D EMA",                  1, 2, -1, -1, -1);  
    expect_pack("32F EMA",                 1, 2, -1, -1, -1);  
    expect_pack("1F DX",                   1, 2, -1, -1, -1);  
    expect_pack("0",                       1, 2, -1, -1, -1);  
    expect_pack("131071",                  1, 2, -1, -1, -1);  
    expect_pack("599 131071",              1, 2, -1, -1, -1);  
    expect_pack("599 001",                 2, -1, -1, -1, -1);  
    expect_pack("599 05",                  2, -1, -1, -1, -1);  
    expect_pack("599 BRUCE",               2, -1, -1, -1, -1);  
    expect_pack("131072",                  2, 3, -1,  3, -1);  
    expect_pack("000001",                  2, 3, -1,  3, -1);  
    expect_pack("01D EMA",                 2, 3, -1,  3, -1);  
    expect_pack("1G EMA",                  2, 3, -1,  3, -1);  
    expect_pack("K1ABC 1D EMAX QSL TU",    4, 0, 1,  3, -1);  
    expect_pack("599 SA00",                2, -1, -1, -1, -1);  
    expect_pack("599 XYZ",                 1, 2, -1, -1, -1);  
    expect_pack("599 0AB",                 2, -1, -1, -1, -1);  
    expect_pack("CQ K1ABC CQ!",            3, -1, -1, -1, -1);  
    expect_pack("TEST WB9XYZ",             2, 3, -1,  0,  1);  
    expect_pack("HI WB9XYZ",               2, 3, -1,  3, -1);  
    expect_pack("K1ABC HELLO",             2, 0, 1,  3, -1);  
    expect_pack("TEST AGN NR",             2, 3, -1,  2, -1);  
    expect_pack("599 A",                   1, 3, -1, -1, -1);  
    expect_pack("A599 MA",                 2, 3, -1,  3, -1);  
    expect_pack("VP2/KF2GHI",              2, 3, -1,  3, -1);  
    expect_pack("QU1RK",                   1, 3, -1, -1, -1);  
    expect_pack("TU QU1RKP CQ",            3, 2, -1,  3, -1);  
    expect_pack("WB9XYZABC",               2, 3, -1,  3, -1);  
    expect_pack("K1A A",                   1, 3, -1, -1, -1);  
    expect_pack("PSE AGN NR",              2, -1, -1, -1, -1);  
    expect_pack("AGN NR PSE",              2, 2, -1,  3, -1);  
    // CONTROL-фразы: каждая должна дать 1 кадр с family=4/misc=CONTROL  
    static const char* ctrl[18] = {  
        "AGN?","CALL?","AGN CALL","NR?","AGN NR","EXCH?","STATE?","SECTION?",  
        "ZONE?","GRID?","RPRT?","QSL TU","TU","QRZ?","QSO B4","WAIT","NIL?","OK?"};  
    for (int i = 0; i < 18; i++) {  
        tp_pass++;  
        JttyFrameList fl;  
        if (!jtty_pack_message(ctrl[i], &fl, JTTY_EXCHANGE_UNKNOWN, -1) || fl.nframes != 1) {  
            Serial.printf("  FAIL ctrl %d \"%s\"\n", i, ctrl[i]); tp_fail++; tp_pass--; continue;  
        }  
        uint32_t n32 = (uint32_t)(fl.frames[0] >> 2);  
        if ((n32 & 3) != 2 || ((n32 >> 2) & 7) != 4 ||  
            ((n32 >> 16) & 0x7F) != (uint32_t)i) {  
            Serial.printf("  FAIL ctrlbits %d \"%s\" n32=%08lX\n", i, ctrl[i], n32);  
            tp_fail++; tp_pass--;  
        }  
    }  
    Serial.printf("[JTTY PACK] %d passed, %d failed\n", tp_pass, tp_fail);  
}





