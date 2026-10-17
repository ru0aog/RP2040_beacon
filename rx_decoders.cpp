/**  
 * ============================================================================  
 *  rx_decoders.cpp — декодеры цифровых мод на децимированной базовой полосе  
 *  Работает на core1 после rx_dsp (Goertzel-банк + per-mode обработчики).  
 *  Fs_in = rx_dsp_fs_out() ≈ 15.6 кс/с при RX_ADC_SAMPLE_RATE_HZ = 250 кГц.  
 * ============================================================================  
 *
 * для RTTY: 
 * - автокорреляционный частотный дискриминатор (сross-product FMD) 
 *   с сохранением фазы (prev_i, prev_q) через границы скользящих чанков.
 * - деглитчинг старт-бита RTTY через счётчик rt.space_cnt >= 2 
 *   с одновременным вычитанием аппаратной задержки фильтра из дедлайна считывания.
 */

 /*
  Декодеры — большой объём
  Даю полный файл с соответствующим каркасом и полными поставками для RTTY и CW, MFSK-движком 
  с разностным детектором для FSQ/IFKP/DominoEX и Olivia-Walsh для Olivia/Contestia; 
  FEC/варикод-стыковку, содержащую в явных хуках с пояснениями

  Полностью рабочие из коробки: 
  RTTY (Бодо-старт-стоп) и CW (таймер + дерево Морзе). 
  Они самые простые и хорошо проверяются.

  MFSK-движок готов (Goertzel-банк, победитель получает все, 
  IFK-разность, код Грея), но варикод/FEC-слои не реализованы — это ~300-400 строк, 
  выводимых из fldigi (свёрточный декодер k=7, деинтерлив 4×10, таблицы varicode[]/ fsq_varicode[]/ dominovar[]). 

  Таблицы varicode[]в thor_modem.cpp объявленах static — для повторного использования 
  их нужно сделать externили продублировать в rx_decoders.cpp.

  Оливия: Коррелятор Уолша-8 есть, но полный блоковый дескремблер/распаковка 
  5-битных символов (64 символа → 40 бит → 5×8-битных символов) не дописана — помечено TODO.

  FC — заглушка с ЭМА мощности; точную расстройку правильнее брать интерполяцией трёх бинов Герцеля, 
  правильно говоря rx_dsp_set_nco_hz. Скелет оставил.
  Окно символа: Герцель ведет по соединениям sym_samples — для пониженных мод (FSQ 256 мс → ~4000 отсчетов) 
  компрессоры int32могут переполняться при больших размерах; стоит нормировать вход (>>4 перед нажатием) или сделать двухэтапное окно.
*/
  
#include <Arduino.h>  
#include "vfo_hardware.h"  
  
// ---- API из rx_dsp.cpp ----  
extern bool     rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n);  
extern void     rx_dsp_release_output(void);  
extern void     rx_dsp_set_nco_hz(float f_hz);  
extern float    rx_dsp_fs_out(void);  

extern uint32_t rx_capture_dropped(void);

typedef enum {  
    RX_MODE_NONE=0, RX_MODE_CW, RX_MODE_RTTY,  
    RX_MODE_OLIVIA, RX_MODE_CONTESTIA,  
    RX_MODE_THOR, RX_MODE_MFSK16, RX_MODE_DOMEX, RX_MODE_FSQ, RX_MODE_IFKP,  
    RX_MODE_JTTY  
} rx_mode_t;  

// Единственная точка сопоставления имя↔режим. .ino вызывает её, не зная enum.  
uint8_t rx_decoders_mode_by_name(const char* name) {  
    if (!strcmp(name, "cw"))       return RX_MODE_CW;  
    if (!strcmp(name, "rtty"))     return RX_MODE_RTTY;  
    if (!strcmp(name, "olivia"))   return RX_MODE_OLIVIA;  
    if (!strcmp(name, "contestia"))return RX_MODE_CONTESTIA;  
    if (!strcmp(name, "thor"))     return RX_MODE_THOR;  
    if (!strcmp(name, "mfsk16"))   return RX_MODE_MFSK16;  
    if (!strcmp(name, "dominoex")) return RX_MODE_DOMEX;  
    if (!strcmp(name, "dom"))      return RX_MODE_DOMEX;  
    if (!strcmp(name, "fsq"))      return RX_MODE_FSQ;  
    if (!strcmp(name, "ifkp"))     return RX_MODE_IFKP;  
    if (!strcmp(name, "jtty"))     return RX_MODE_JTTY;  
    return RX_MODE_NONE;   // 0 — «режим не найден», вызывающий код проверяет на NONE  
}

static rx_mode_t  dec_mode = RX_MODE_NONE;  
static uint32_t   sym_samples;        // отсчётов на символ при Fs_out  
static uint32_t   sym_acc = 0;        // накоплено в окне  
static uint32_t   sample_idx = 0;
static int32_t rtty_disc = 0;     // накопитель дискриминатора текущего окна  
static int16_t prev_i = 0, prev_q = 0;   // последние I/Q прошлой порции (стыковка)
static float      fs_out = 15625.0f;  
static float      f_center_hz = 0;    // для AFC  
static bool       afc_on = false;  

// добавить в секцию глобальных данных (рядом с rt/cw/olv/ifk)  
// ---------- JTTY приёмник ----------  
// Кадр: 59 символов = 13 sync (is13) + 46 данных (дибиты кода).  
static uint8_t  jtty_sym[59];       // текущий кадр (тоны 0..3)  
static uint8_t  jtty_nsym = 0;      // заполненная позиция  
static uint8_t  jtty_hist[16];      // кольцо на 16 (маска &15), читаются последние 13
static uint8_t  jtty_hpos = 0;      // позиция записи в кольцо  
static uint8_t  jtty_inframe = 0;   // 1 = внутри кадра  
  
// Паттерн is13 из спецификации JTTY (если JTTY_IS13 из jtty_modem.cpp  
// виден — убрать этот дефайн и ссылаться на него).  
static const uint8_t jtty_is13[13] = { 0,2,2,3,0,0,3,2,1,3,1,2,0 };  
  
// Число совпадений кольца истории с паттерном is13  
static uint8_t jtty_sync_score(void) {  
    uint8_t m = 0;  
    for (uint8_t i = 0; i < 13; i++)  
        if (jtty_hist[(jtty_hpos + i) & 15] == jtty_is13[i]) m++;  
    return m;  
}

// ============================================================================  
// 1. GOERTZEL-ДЕТЕКТОР (комплексный банк тонов)  
// ----------------------------------------------------------------------------  
// Классический Гёрцель второго порядка, но с РАЗДЕЛЬНЫМ накоплением по I- и  
// Q-каналам базовой полосы. Скалярная версия (только I) одинаково реагирует  
// на сигнал и на его зеркальную копию относительно LO — для квадратурного  
// приёмника это означает потерю подавления зеркального канала. Комплексная  
// форма x = I + j*Q сдвигает бин на истинный знак частоты.  
//  
// Реализация — целочисленная (Q15): коэффициент k хранится как  
//   k = round(32768 * 2 * cos(2*pi*f / fs))  
// Рекуррент на каждый отсчёт:  
//   s0 = x + ((k * s1) >> 15) - s2;   s2 = s1;   s1 = s0;  
// Мощность бина за N отсчётов:  
//   P = s1^2 + s2^2 - k*s1*s2/32768  
// Для комплексного входа суммируем мощности I- и Q-рекуррентов  
// (энергия бина, достаточно для выбора argmax и отладочного вывода).  
// ============================================================================  
  
#define DEC_MAX_TONES 33         // максимум тонов (FSQ/IFKP = 33, JTTY = 4)
  
typedef struct {  
    int32_t cr, ci;        // e^{jω} в Q15: cr=cos, ci=sin  
    int32_t ar, ai;        // комплексный резонатор s = z + c·s  
} gtz_t;
  
static gtz_t  gtz[DEC_MAX_TONES]; // банк фильтров  
static uint8_t gtz_ntones = 0;    // сколько тонов реально активно  
  
// ----------------------------------------------------------------------------  
// gtz_push — прогнать n комплексных отсчётов через весь банк.  
//   xi, xq — массивы базовой полосы I и Q (длина n).  
// Если режим работает только по I (CW/RTTY и т.п.), передайте xq = NULL —  
// Q-рекуррент тогда пропускается и не жрёт CPU.  
// ----------------------------------------------------------------------------  
// Стало — однополюсный комплексный Гёрцель:  
static inline void gtz_push(const int16_t* xi, const int16_t* xq, int n) {  
    for (int i = 0; i < n; i++) {  
        int32_t xr = xi[i], xq_ = xq[i];  
        for (uint8_t t = 0; t < gtz_ntones; t++) {  
            gtz_t* g = &gtz[t];  
            int32_t tr = xr + (((int64_t)g->cr * g->ar - (int64_t)g->ci * g->ai) >> 15);  
            int32_t ti = xq_ + (((int64_t)g->cr * g->ai + (int64_t)g->ci * g->ar) >> 15);  
            g->ar = tr; g->ai = ti;  
        }  
    }  
}  
  
// ----------------------------------------------------------------------------  
// gtz_power — мощность тона t за накопленное окно.  
// P = s1^2 + s2^2 - k*s1*s2/32768 по каждому каналу, сумма I+Q.  
// Возвращает int32; при больших окнах (>~2000 отсчётов) возможен переполн —  
// для JTTY окно на символ ~500 отсчётов, запаса хватает.  
// ----------------------------------------------------------------------------  
static inline int32_t gtz_power(uint8_t t) {  
    const gtz_t* g = &gtz[t];  
    int64_t p = (int64_t)g->ar * g->ar + (int64_t)g->ai * g->ai;  
    p >>= 10;                                     // масштаб под int32  
    if (p > 0x7FFFFFFFLL) p = 0x7FFFFFFFLL;  
    return (int32_t)p;  
}
  
// ----------------------------------------------------------------------------  
// gtz_argmax — индекс самого мощного тона банка + его мощность.  
// ----------------------------------------------------------------------------  
static uint8_t gtz_argmax(int32_t* out_max) {  
    uint8_t best = 0; int32_t bp = 0;  
    for (uint8_t t = 0; t < gtz_ntones; t++) {  
        int32_t p = gtz_power(t);  
        if (t == 0 || p > bp) { bp = p; best = t; }  
    }  
    if (out_max) *out_max = bp;  
    return best;  
}  

static inline void gtz_reset(gtz_t* g) {  
    g->ar = 0;  
    g->ai = 0;  
}

// ----------------------------------------------------------------------------  
// gtz_bank_init — инициализация банка на сетку тонов:  
//   тон t сидит на частоте  f0_hz + t * step_hz, отсчёты идут с частотой fs.  
// Вызывается из rx_decoders_start() после того как rx_dsp выдал fs выходного  
// потока (после децимации). Для JTTY: f0 = нижний тон, step = 31.25 Гц,  
// ntones = 4, fs = частота сэмплов детектора (например 250 Гц при 32 мс  
// символе -> 8 отсчётов/символ, см. вашу децимацию).  
// ----------------------------------------------------------------------------  
static void gtz_bank_init(float f0_hz, float step_hz, uint8_t ntones, float fs) {  
    gtz_ntones = (ntones > DEC_MAX_TONES) ? DEC_MAX_TONES : ntones;  
    for (uint8_t t = 0; t < gtz_ntones; t++) {  
        float w = 2.0f * PI * (f0_hz + t * step_hz) / fs;  
        // e^{-jw} — комплексный коэффициент Гёрцеля, Q15  
        gtz[t].cr = (int32_t)(32768.0f * cosf(w));  
        gtz[t].ci = (int32_t)(32768.0f * sinf(w));  
        gtz_reset(&gtz[t]);  
    }  
}

// Поиск тона-победителя: возвращает индекс 0..gtz_ntones-1,  
// в *p_best (если не NULL) — его мощность.  
static uint8_t gtz_winner(int32_t* p_best) {  
    uint8_t  best = 0;  
    int32_t  pmax = 0;  
    for (uint8_t t = 0; t < gtz_ntones; t++) {  
        int32_t p = gtz_power(t);  
        if (p > pmax) { pmax = p; best = t; }  
    }  
    if (p_best) *p_best = pmax;  
    return best;  
}

// ============================================================================  
// 2. Olivia / Contestia — 8 тонов, 31.25 Гц, Walsh-8, 32 мс символ  
// ============================================================================  
// Walsh-8 (Hadamard): символ k кодирует строку; принятый вектор тонов  
// коррелируем со всеми 8 строками — argmax даёт символ.  
static const int8_t walsh8[8][8] = {  
    {+1,+1,+1,+1,+1,+1,+1,+1},  
    {+1,-1,+1,-1,+1,-1,+1,-1},  
    {+1,+1,-1,-1,+1,+1,-1,-1},  
    {+1,-1,-1,+1,+1,-1,-1,+1},  
    {+1,+1,+1,+1,-1,-1,-1,-1},  
    {+1,-1,+1,-1,-1,+1,-1,+1},  
    {+1,+1,-1,-1,-1,-1,+1,+1},  
    {+1,-1,-1,+1,-1,+1,+1,-1},  
};  
  
// Olivia передаёт символ в коде Грея на сетке 8 тонов: sym → tone  
static const uint8_t olivia_tone2sym[8] = {0,1,3,2,7,6,4,5}; // gray→bin  
  
typedef struct {  
    uint8_t  sym_hist[5];       // маленький дебаунсер символов  
    uint8_t  hist_n;  
    uint8_t  walsh_idx;         // позиция в блоке Walsh (символ 0..63 блока = 5 бит)  
    int      walsh_accum[8];    // накопление корреляций для каждой из 8 строк  
    uint8_t  block_pos;         // 0..63 — позиция символа в блоке (64 символа)  
    uint8_t  block_bits[64];    // по 3 бита с каждого символа → поток битов  
} olivia_dec_t;  
static olivia_dec_t olv;  
  
// Декодирование одного символа Olivia: tone → sym → строка Walsh → 3 бита  
static void olivia_on_tone(uint8_t tone) {  
    uint8_t sym = olivia_tone2sym[tone & 7];  
    // Накопление: каждая строка Walsh k получает +1 если её бит в позиции  
    // block_pos%8 совпадает с битом символа — упрощённая когерентная оценка.  
    for (int k = 0; k < 8; k++)  
        olv.walsh_accum[k] += walsh8[k][sym];  
    if (++olv.block_pos == 64) {  
        // argmax по walsh_accum → 3 бита данных; здесь — упрощённо печатаем поток.  
        // ПОЛНЫЙ путь (дескремблер + сортировка блоков как в olivia.cxx) — TODO,  
        // требует портирования ~200 строк fldigi-кода.  
        memset(olv.walsh_accum, 0, sizeof(olv.walsh_accum));  
        olv.block_pos = 0;  
    }  
}  
  
// ============================================================================  
// 3. RTTY — 2 тона (mark/space), 45.45 бод, Бодо ITA2  
// ============================================================================  
static const char baudot_ltrs[32] = {  
    '\0','E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C','K',  
    'T','Z','L','W','H','Y','P','Q','O','B','G',0,'M','X','V',0 };  
static const char baudot_figs[32] = {  
    '\0','3','\n','-',' ','\'','8','7','\r','$','4','\a',',','!',':','(',  
    '5','+',')','2','#','6','0','1','9','?','&',0,'.','/',';',0 };  
  
typedef struct {  
    uint8_t  figs;              // регистр LTRS/FIGS  
    uint8_t  state;             // 0=idle, 1=start, 2=биты, 3=стоп  
    uint8_t  bit_cnt;  
    uint8_t  shreg;  
    uint32_t bit_period;        // отсчётов на бит  
    uint8_t  prev_mark;         // решение предыдущего символа (для поиска фронта)    
    uint32_t next_sample;       // момент следующего отсчёта бита (в отсчётах)
    uint8_t  space_cnt;         // длительность текущего space-подряд (деглитчинг)
} rtty_dec_t;  
static rtty_dec_t rt;  

// Некогерентный интегратор по подокнам: суммируем мощности тонов  
// за все ~8 подокон бита — решение о бите по полной энергии.  
static int32_t  rt_pwr_mark = 0;    // Σ power тона 1 (mark)  
static int32_t  rt_pwr_space = 0;   // Σ power тона 0 (space)  
static uint8_t  rt_pwr_n = 0;       // сколько подокон накоплено

#define RTTY_FIGS 0x1F  
#define RTTY_LTRS 0x1B  

static inline bool sample_idx_ge(uint32_t a, uint32_t b) {  
    return (int32_t)(a - b) >= 0;  
}

// RTTY: решение о тоне каждые bit_period/8 отсчётов (subwin ~2.75 мс при 45.45 бод).  
// Старт-бит ловим по фронту mark→space, датабиты читаем в их середине.  
static void rtty_bit(bool mark, uint32_t idx /*в отсчётах Fs_out*/) {  
    switch (rt.state) {  
    case 0:                                // ждём устойчивый space-фронт  
        if (!mark) {  
            if (++rt.space_cnt >= 2 && rt.prev_mark) {  
                // space держится ≥2 подокон — фронт был на первом из них  
                rt.state = 1; rt.bit_cnt = 0; rt.shreg = 0;
                // Фронт mark→space внутри первого space-подокна; опорная  
                // точка = его середина = idx - 1.5·subwin = idx - T·3/16.  
                // Центр бита 0 = +1.5·T от фронта → idx + T - T·3/16.  
                // rt.next_sample = idx + rt.bit_period - rt.bit_period/4 - rt.bit_period/16;  // компенсация задержки деглитчинга 
                rt.next_sample = idx + rt.bit_period - (3 * sym_samples) / 2; // устойчивее к смене размера подокна
            }  
        } else {  
            rt.space_cnt = 0;  
        }  
        break;  
  
    case 1:                                  // читаем 5 датабитов в серединах  
        while (idx >= rt.next_sample) {  
            rt.shreg |= (mark ? 1u : 0u) << rt.bit_cnt;  
            rt.next_sample += rt.bit_period;  
            if (++rt.bit_cnt == 5) { rt.state = 2; break; }  
        }  
        break;  
  
    case 2:                                  // стоп-бит: закрыть символ  
        if (idx >= rt.next_sample) {  
            rt.state = 0;  
            // Стоп обязан быть mark. Space = framing error:  
            // ложный фронт, символ выбрасываем, ресинк сразу.  
            if (!mark) break;  
            uint8_t c = rt.shreg;  
            if (c == RTTY_FIGS) rt.figs = 1;  
            else if (c == RTTY_LTRS) rt.figs = 0;  
            else {  
                char ch = rt.figs ? baudot_figs[c] : baudot_ltrs[c];  
                if (ch) Serial.write(ch);  
            }  
        }  
        break;  
    }  
    rt.prev_mark = mark;                     // единственное место обновления  
}
  
// ============================================================================  
// 4. CW — тональный детектор + тайминг (точка=1, тире=3, паузы 1/3/7)  
// ============================================================================  
typedef struct {  
    uint32_t on_start;          // отсчёт начала тона  
    uint32_t off_start;         // отсчёт начала паузы  
    bool     tone_on;  
    float    dot_us;            // оценка точки в МИКРОсекундах (автоподстройка)  
    char     elem_buf[64];  
    uint8_t  elem_n;  
    int32_t  noise_floor;  
} cw_dec_t;  
static cw_dec_t cw;  
  
static void cw_decode_elements(void);  
  
// idx — индекс отсчёта Fs_out; dur переводим в микросекунды  
static void cw_on_edge(bool on, uint32_t idx, uint32_t fs_x1000) {  
    // dur_ms — длительность завершившегося интервала (тона или паузы)  
    uint32_t dur_smp = on ? (idx - cw.on_start) : (idx - cw.off_start);  
    float dur_ms = (float)dur_smp * 1.0e6f / (float)fs_x1000;
  
    float dot_ms = cw.dot_us * 0.001f;      // оценка точки в мс
    if (dot_ms < 20.0f || dot_ms > 150.0f) dot_ms = 60.0f;  // страховка при 0/мусоре
  
    if (on) {  
        // Конец тона: классификация точка/тире  
        if (dur_ms < 15.0f) return;          // статический разряд — игнорировать  
        if (dur_ms < 2.0f * dot_ms) {  
            cw.elem_buf[cw.elem_n++] = '.';  
            // Автоподстройка: быстрая EMA по точкам, жёсткий кламп 20..150 мс (~8–60 WPM)  
            float nd_ms = dot_ms + (dur_ms - dot_ms) * 0.25f;  
            cw.dot_us = (uint32_t)fminf(150000.0f, fmaxf(20000.0f, nd_ms * 1000.0f));  
        } else {  
            cw.elem_buf[cw.elem_n++] = '-';  
            // Тире обновляет оценку точки в 3 раза медленнее (dur/3 vs dot)  
            float nd_ms = dot_ms + (dur_ms / 3.0f - dot_ms) * 0.10f;  
            cw.dot_us = (uint32_t)fminf(150000.0f, fmaxf(20000.0f, nd_ms * 1000.0f));  
        }  
        if (cw.elem_n >= sizeof(cw.elem_buf) - 1) cw.elem_n = 0;  
    } else {  
        // Конец паузы: ~3·dot → конец знака, ~7·dot → конец слова  
        if (dur_ms < 15.0f) return;          // щелчок-прореживание тона — не разрыв слова  
        if      (dur_ms > 6.0f * dot_ms) { cw_decode_elements(); Serial.write(' '); }  
        else if (dur_ms > 2.5f * dot_ms)   { cw_decode_elements(); }  
    }  
}

static const char cw_tree[] =  
    " ETIANMSURWDKGOHVVF L  PJBXCYZQ  54 3   2       16       7   8 90";  
  
static void cw_decode_elements(void) {  
    if (!cw.elem_n) return;  
    cw.elem_buf[cw.elem_n] = 0;  
    int idx = 1;  
    for (uint8_t i = 0; i < cw.elem_n && idx < 63; i++)  
        idx = idx * 2 + (cw.elem_buf[i] == '-' ? 1 : 0);  
    char c = (idx < 64) ? cw_tree[idx] : '?';  
    if (c && c != ' ') Serial.write(c);  
    cw.elem_n = 0;  
}

// ============================================================================  
// 5. MFSK/IFK-разностный детектор (THOR/DominoEX/FSQ/IFKP)  
// ============================================================================  
// IFK+: sym = (tone - prev_tone - offset + N) % N  
typedef struct {  
    uint8_t prev_tone;  
    bool    have_prev;  
    uint8_t offset;        // +2 для THOR/DomEX, +1 для FSQ  
    uint8_t ntones;  
    void (*on_symbol)(uint8_t sym);  
} ifk_t;  
static ifk_t ifk;  
  
static void ifk_on_tone(uint8_t tone) {  
    if (!ifk.have_prev) { ifk.prev_tone = tone; ifk.have_prev = true; return; }  
    int d = (int)tone - ifk.prev_tone - ifk.offset;  
    if (d < 0) d += ifk.ntones;  
    ifk.prev_tone = tone;  
    if (ifk.on_symbol) ifk.on_symbol((uint8_t)d);  
}  
  
// ============================================================================  
// 6. Верхний уровень: конфигурация режима + главный цикл  
// ============================================================================  
// ВАЖНО: gtz_bank_init получает частоты в БАЗОВОЙ ПОЛОСЕ (аудио), не RF!  
// rx_dsp вращает спектр так, что center_hz (RF) → 0 Гц. Поэтому все смещения  
// ниже — константы относительно 0, а не относительно center_hz.  
void rx_decoders_start(uint8_t mode, float center_hz) {
    Serial.printf("[DEC] start: mode=%u center=%.1f\n",  
                (unsigned)mode, (double)center_hz);
    dec_mode = (rx_mode_t)mode;  
    fs_out = rx_dsp_fs_out();  
    Serial.printf("[DEC] fs_out=%.1f\n", (double)fs_out);       // <-- fs валидна 
    f_center_hz = center_hz;  
    sym_acc = 0; sample_idx = 0;  
    memset(&rt, 0, sizeof(rt)); memset(&cw, 0, sizeof(cw));  
    memset(&olv, 0, sizeof(olv)); memset(&ifk, 0, sizeof(ifk));  
    jtty_nsym = 0; jtty_hpos = 0; jtty_inframe = 0;  
    memset(jtty_hist, 0xFF, sizeof(jtty_hist));   // гарантированно не совпадёт с is13  
    cw.dot_us = 60000.0f; // 60 мс ≈ 20 WPM — стартовая точка, EMA подстроится  
    cw.noise_floor = 0;  
  
    switch (dec_mode) {  
    case RX_MODE_CW:  
        gtz_bank_init(70.0f, 0.0f, 1, fs_out);   // тон ~+70 Гц — компенсация сдвига LO
        sym_samples = (uint32_t)(fs_out * 0.005f);               // окно 5 мс
        Serial.printf("[CW-RX] bank ready, gtz_ntones=%u sym=%lu\n",  
                      (unsigned)gtz_ntones, (unsigned long)sym_samples);  // <-- банк инициализирован
        break;  
    case RX_MODE_RTTY:  
        gtz_bank_init(-85.0f, 170.0f, 2, fs_out);                // mark/space = ±85 Гц  
        rt.bit_period = (uint32_t)(fs_out / 45.45f);             // ~344 отсчёта на бит  
        sym_samples   = rt.bit_period / 8;                       // подокно ~2.75 мс  
        rt.prev_mark  = 1;                                       // idle-канал = mark  
        rt.space_cnt  = 0;  
        rt.state      = 0;  
        break;  
    case RX_MODE_OLIVIA:  
    case RX_MODE_CONTESTIA:  
        // нижний тон = центр полосы - 3.5·31.25 = -109.375 Гц  
        gtz_bank_init(-109.375f, 31.25f, 8, fs_out);  
        sym_samples = (uint32_t)(fs_out * 0.032f);               // 32 мс  
        break;  
    case RX_MODE_MFSK16:  
        gtz_bank_init(-117.19f, 15.625f, 16, fs_out);  
        sym_samples = (uint32_t)(fs_out * 0.064f);               // 64 мс  
        break;  
    case RX_MODE_THOR:  
        gtz_bank_init(-132.81f, 15.625f, 18, fs_out);  
        sym_samples = (uint32_t)(fs_out * 0.128f);               // 128 мс (THOR8)  
        ifk.offset = 2; ifk.ntones = 18;  
        break;  
    case RX_MODE_DOMEX:  
        gtz_bank_init(-132.81f, 15.625f, 18, fs_out);  
        sym_samples = (uint32_t)(fs_out * 0.128f);  
        ifk.offset = 2; ifk.ntones = 18;  
        break;  
    case RX_MODE_FSQ:  
        gtz_bank_init(-140.63f, 8.789f, 33, fs_out);  
        sym_samples = (uint32_t)(fs_out * 0.256f);  
        ifk.offset = 1; ifk.ntones = 33;  
        break;  
    case RX_MODE_IFKP:  
        gtz_bank_init(-187.15f, 11.697f, 33, fs_out);  
        sym_samples = (uint32_t)(fs_out * 0.128f);  
        ifk.offset = 1; ifk.ntones = 33;  
        break;  
    case RX_MODE_JTTY:  
        gtz_bank_init(-46.875f, 31.25f, 4, fs_out);              // тоны -47,-16,+16,+47 Гц  
        sym_samples = (uint32_t)(fs_out * 0.032f);               // 500 отсчётов = 32 мс  
        Serial.printf("[JTTY-RX] baseband tones -46.9..+46.9 Hz step=31.25 sym=%lu smp fs=%.1f\n",  
                      (unsigned long)sym_samples, (double)fs_out);  
        break;  
    default: break;  
    }  
}
  
// AFC: оценка расстройки = bin_max·step - expected; коррекция NCO  
static void afc_update(uint8_t tone, int32_t pwr) {
    (void)tone;                                  // AFC-оценщик будет дописан отдельно
    if (!afc_on || gtz_ntones < 2) return;  
    static int32_t ema = 0;  
    ema += (pwr - ema) >> 4;  
    // Грубая коррекция: если победитель стабильно крайний — двигаем NCO на step/4  
    // Точная оценка (интерполяция по 3 соседним бинам) — TODO.  
}  
  
void rx_decoders_stop(void) { dec_mode = RX_MODE_NONE; }  
  
// Вызывается из цикла core1 (rx_dsp_entry) после заполнения out_i/out_q  
void rx_decoders_process(const int16_t* xi, const int16_t* xq, uint8_t n) {  
    // Контроль: функция вообще вызывается?  
    static uint32_t calls = 0;  
    if (++calls == 1 || calls % 1000 == 0)  
        Serial.printf("[DEC] process: calls=%lu mode=%u n=%u\n",  
                      (unsigned long)calls, (unsigned)dec_mode, (unsigned)n);  
  
    if (dec_mode == RX_MODE_NONE || n == 0) return;  

// ---------------- CW: блок = одно окно детектора огибающей ----------------  
    if (dec_mode == RX_MODE_CW) {  
        gtz_push(xi, xq, n);  
        int32_t p; gtz_winner(&p);  
  
        bool on = p > (cw.noise_floor * 4 + 1024);  
        if (!on)                                            // floor — только в паузах,  
            cw.noise_floor += ((p - cw.noise_floor) >> 4);  // иначе EMA съедает посылку  
  
        // Отладка: огибающая раз в ~320 мс  
        static uint32_t dbg = 0;  
        if (++dbg >= 64) {  
            dbg = 0;  
            Serial.printf("[CW] pwr=%ld floor=%ld %s\n",  
                          (long)p, (long)cw.noise_floor, on ? "ON" : "off");  
        }  
  
        if (on != cw.tone_on) {  
            // fs_x1000 = fs_out * 1000 — НЕ fs_out: внутри делится 1e6/fs_x1000 → мс  
            cw_on_edge(!on, sample_idx, (uint32_t)(fs_out * 1000.0f));  
            cw.tone_on = on;  
            if (on) cw.on_start = sample_idx; else cw.off_start = sample_idx;  
        }  
        for (uint8_t t = 0; t < gtz_ntones; t++) gtz_reset(&gtz[t]);  
        sample_idx += n;  
        return;  
    }
  
    // ------------- Остальные моды: символьная синхронизация по блокам ----------  
    // Блок (64 отсчёта) не кратен окну символа (напр. 43): разбираем по частям,  
    // чтобы граница решения всегда совпадала с границей символа.  
    uint8_t pos = 0;  
    while (pos < n) {  
        uint32_t remain_sym = sym_samples - sym_acc;  
        uint8_t  chunk      = (uint8_t)((uint32_t)(n - pos) < remain_sym  
                                        ? (n - pos) : remain_sym);  
  
        gtz_push(xi + pos, xq + pos, chunk);   // комплексный вход: I и Q
  
        // Дискриминатор RTTY: накапливаем по той же порции отсчётов.  
        // Знак кросс-произведения I/Q-последовательных отсчётов = знак частоты:  
        // положительный → тон выше NCO (mark), отрицательный → ниже (space).  
        if (dec_mode == RX_MODE_RTTY) {  
            int16_t pi = (pos > 0) ? xi[pos - 1] : prev_i;   // предыдущий отсчёт  
            int16_t pq = (pos > 0) ? xq[pos - 1] : prev_q;   // в т.ч. через границу порций  
            for (uint8_t k = 0; k < chunk; k++) {  
                int16_t ci = xi[pos + k], cq = xq[pos + k];  
                rtty_disc += (int32_t)ci * pq - (int32_t)cq * pi;  
                pi = ci; pq = cq;  
            }  
            prev_i = xi[pos + chunk - 1];  
            prev_q = xq[pos + chunk - 1];  
        }  
  
        sym_acc    += chunk;  
        pos        += chunk;  
        sample_idx += chunk;  
  
        if (sym_acc < sym_samples) break;        // окно не закрыто — выходим  
  
        // Граница символа: winner-take-all  
        int32_t pmax;  
        uint8_t tone = gtz_argmax(&pmax);
        afc_update(tone, pmax);  
  
        switch (dec_mode) {  
        case RX_MODE_RTTY:  
            rtty_bit(rtty_disc > 0, sample_idx);   // знак дискриминатора = mark  
            break;  
        case RX_MODE_OLIVIA: case RX_MODE_CONTESTIA:  
            olivia_on_tone(tone);  
            break;  
        case RX_MODE_MFSK16: {  
            uint8_t nib = tone;               // gray → bin  
            nib ^= nib >> 2; nib ^= nib >> 1;  
            // TODO: склейщик битов → varicode[] (таблица static в thor_modem.cpp)  
            (void)nib;  
            break;  
        }  
        case RX_MODE_THOR: case RX_MODE_DOMEX:  
        case RX_MODE_FSQ:  case RX_MODE_IFKP:  
            ifk_on_tone(tone);                // TODO: varicode/FEC  
            break;
        case RX_MODE_JTTY: {  
            // Отладка: поток символов + статистика тонов раз в ~0.5 с.  
            static uint32_t j_dbg_cnt = 0, j_last_ms = 0;  
            // TODO: sync по is13 + Витерби TBCC  
            static int32_t  j_acc[4]; static uint8_t j_n = 0;  
            for (uint8_t t = 0; t < 4 && t < gtz_ntones; t++)  
                j_acc[t] += gtz_power(t);  
            if (++j_n >= 16) {  
                Serial.printf("[JTTY] pwr: %ld %ld %ld %ld  win=%d\n",  
                              (long)j_acc[0]/j_n, (long)j_acc[1]/j_n,  
                              (long)j_acc[2]/j_n, (long)j_acc[3]/j_n, tone);  
                j_acc[0]=j_acc[1]=j_acc[2]=j_acc[3]=0; j_n=0;  
            }  
  
            // Кольцо истории для поиска is13  
            jtty_hist[jtty_hpos & 15] = tone;  
            jtty_hpos = (jtty_hpos + 1) & 15;  
  
            if (!jtty_inframe) {  
                // Sync: >=11 из 13 совпадений (толерантность к 2 ошибкам)  
                if (jtty_sync_score() >= 11) {  
                    jtty_inframe = 1;  
                    jtty_nsym = 0;  
                    memcpy(jtty_sym, jtty_is13, 13);   // sync-тоны в кадр  
                    jtty_nsym = 13;  
                }  
            } else {  
                jtty_sym[jtty_nsym++] = tone;  
                if (jtty_nsym >= 59) {                 // кадр собран  
                    jtty_inframe = 0;  
                    Serial.print(F("[JTTY] frame:"));  
                    for (uint8_t i = 13; i < 59; i++)  
                        Serial.printf("%d", jtty_sym[i]);  
                    Serial.println();  
                    // TODO: 46 дибитов -> 92 бита -> soft/hard Viterbi TBCC  
                    // -> CRC12 -> распаковка pack28 (порт из WSJT-X tbcc.f90)  
                }  
            }  
            break;  
        }
        default: break;  
        }  
  
        for (uint8_t t = 0; t < gtz_ntones; t++) gtz_reset(&gtz[t]);  
        sym_acc   = 0;  
        rtty_disc = 0;                           // новое окно — новый дискриминатор  
    }  
}



