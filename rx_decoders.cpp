/**    
 * ============================================================================    
 *  rx_decoders.cpp — декодеры цифровых мод на децимированной базовой полосе    
 *  Работает на core1 после rx_dsp (Goertzel-банк + per-mode обработчики).    
 *  Fs_in = rx_dsp_fs_out() ≈ 15.6 кс/с при RX_ADC_SAMPLE_RATE_HZ = 250 кГц.    
 * ============================================================================    
 *  
 * для RTTY (порт ядра fldigi rtty.cxx):  
 * - два комплексных миксера переносят mark/space в нулевую частоту;  
 * - согласованный НЧ-фильтр = скользящее среднее длиной в бит (boxcar);  
 * - детекторы огибающей decayavg + ATC-слайсер v3 (v1 1/16, v2 1/32, v3 1/64);  
 * - старт-стопный битовый таймер с деглитчингом старт-бита;  
 * - AFC: остаточное вращение согласованного фильтра за бит → поправка миксеров.  
 */  
  
/**  
 * ============================================================================  
 *  rx_decoders.cpp (CW Блок) — Амплитудный некогерентный CW-декодер  
 *  Архитектура: Двухъядерная обработка (выполняется на core1 после rx_dsp)  
 *  Входной поток: Квадратурные отсчеты (I/Q), Fs_in ≈ 15.6 кс/с  
 * ============================================================================  
 *  
 *  1. ФИЗИЧЕСКИЙ УРОВЕНЬ: одиночный комплексный бин Гёрцеля на +63 Гц,  
 *     EMA огибающей ~33 мс, асимметричный AGC-трекер fldigi (шум >>8/>>3,  
 *     сигнал >>2/>>9), гистерезис 3/5 и 2/5 динамического диапазона.  
 *  2. ЛОГИЧЕСКИЙ УРОВЕНЬ (UHSDR): дебаунс спада 3 окна, адаптивные  
 *     cw_dot_avg/cw_dash_avg, композитный порог, бинарное дерево Морзе.  
 * ============================================================================  
 */  
  
#include <Arduino.h>    
#include "vfo_hardware.h"    


// ---- API из rx_dsp.cpp ----    
extern bool     rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n);    
extern void     rx_dsp_release_output(void);    
extern void     rx_dsp_set_nco_hz(float f_hz);    
extern float    rx_dsp_fs_out(void);    
  
extern uint32_t rx_capture_dropped(void);  
static void rtty_emit(uint8_t c);

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
    return RX_MODE_NONE;    
}  
  
bool              dbg_rx = true;     // флаг печати лога отладки  
  
static rx_mode_t  dec_mode = RX_MODE_NONE;    
static uint32_t   sym_samples;        // отсчётов на символ при Fs_out    
static uint32_t   sym_acc = 0;        // накоплено в окне    
static uint32_t   sample_idx = 0;  
static float      fs_out = 15625.0f;    
static float      f_center_hz = 0;    // для AFC    
static bool       afc_on = false;  
static bool rtty_scan = false;

// ---------- JTTY приёмник ----------    
// Кадр: 59 символов = 13 sync (is13) + 46 данных (дибиты кода).    
static uint8_t  jtty_sym[78];       // текущий кадр (тоны 0..3)    
static uint8_t  jtty_nsym = 0;      // заполненная позиция    
static uint8_t  jtty_hist[16];      // кольцо на 16 (маска &15), читаются последние 13  
static uint8_t  jtty_hpos = 0;      // позиция записи в кольцо    
static uint8_t  jtty_inframe = 0;   // 1 = внутри кадра    
static uint8_t  jtty_state  = 0;   // 0/1/2 — автомат альтернации дибитов  
static uint8_t  jtty_symcnt = 0;   // накоплено дибитов в кадре

// Паттерн is13 из спецификации JTTY  
static const uint8_t jtty_is13[13] = { 0,2,2,3,0,0,3,2,1,3,1,2,0 };    
    
// Число совпадений кольца истории с паттерном is13    
static uint8_t jtty_sync_score(void) {    
    uint8_t m = 0;    
    for (uint8_t i = 0; i < 13; i++)    
        if (jtty_hist[(jtty_hpos + i) & 15] == jtty_is13[i]) m++;    
    return m;    
}  
  
// ---------- CW глобальные данные ----------  
static int32_t  cw_sig_avg    = 0;      // EMA уровня тона (fldigi sig_avg)    
static int32_t  cw_noise_avg = 1024;    // оценка шума огибающей    
static float    cw_dot_avg    = 60.0f;  // мс, адаптивная оценка точки    
static float    cw_dash_avg   = 180.0f; // мс, адаптивная оценка тире    
static float    cw_gap_avg  = 60.0f;    // мс, пауза между элементами    
  
// Дебаунс ключа: off подтверждается только после N подряд «тихих» окон    
static uint8_t  cw_low_cnt  = 0;      // счётчик дебаунса спада    
static int32_t  cw_p_ema    = 0;      // сглаженная огибающая (EMA ~8 окон)  
  
// Состояние CW-декодера    
typedef struct {    
    bool     tone_on;                 // дебаунснутое состояние ключа    
    uint32_t on_start, off_start;     // sample_idx начала тона/паузы    
    uint8_t  elem_n;                  // элементов в буфере знака    
    char     elem_buf[12];            // '.'/'-' текущего знака    
} cw_state_t;    
    
static cw_state_t cw;    



// ================= RTTY: порт rtty::rx_process/rx из fldigi =================  
// cmplx z заменяем на float-пару; fftfilt -> boxcar MA длиной symbollen  
  
#define RTTY_MAX_SYMLEN 512           // хватит под symbollen=344  
  
typedef struct {  
    float fs;                 // входная частота дискретизации (fs_out)  
    float symbollen_f;        // fs/baud (343.8)  
    int   symbollen;          // округлённое, 344  
    int   nbits;              // 5  
    int   stoplen;            // 1.5 бита, отсчётов — для печати/контроля  
  
    // миксеры (n_мод rtty::mixer): фазы и комплексные шаги  
    float mark_phase, space_phase;  
    float f_mark, f_space;    // текущие центры тонов (AFC их сдвигает)  
  
    // boxcar-фильтры (замена fftfilt)  
    float buf_mi[RTTY_MAX_SYMLEN], buf_mq[RTTY_MAX_SYMLEN];  
    float buf_si[RTTY_MAX_SYMLEN], buf_sq[RTTY_MAX_SYMLEN];  
    int   buf_idx;  
    float sum_mi, sum_mq, sum_si, sum_sq;  
  
    // огибающие и шум (rtty.cxx:679-693)  
    float mark_mag, space_mag;  
    float mark_env, space_env;  
    float mark_noise, space_noise;  
    float noise_floor;  
  
    // сдвиговый регистр решений + автомат rx()  
    bool  bit_buf[RTTY_MAX_SYMLEN];  
    int   rxstate;            // 0=IDLE 1=START 2=DATA 3=STOP  
    int   counter;  
    int   bitcntr;  
    int   rxdata;  
    int   figs;               // регистр LTRS/FIGS  
    char  lastchar;  
  
    // AFC (rtty.cxx:836-850): история baseband-комплексов для смежных битов  
    float h_m_re[2], h_m_im[2], h_s_re[2], h_s_im[2];   // [0]=старее  
    float freqerr;  
    int   sym_since_char;                             // отсчёты от последнего char  
} rtty_t;  
  
static rtty_t rrx;  

// rtty.cxx:381-389  mixer  
static inline void rtty_mixer(float &phase, float f, float xr, float xq,  
                              float &or_, float &oi)  
{  
    float c = cosf(phase), s = sinf(phase);  
    or_ = xr * c + xq * s;          // conj-вращение, как в оригинале  
    oi  = xq * c - xr * s;  
    phase += 2.0f * (float)M_PI * f / rrx.fs;  
    if (phase >  2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;  
    if (phase < -2.0f * (float)M_PI) phase += 2.0f * (float)M_PI;
}  
  
static inline float rtty_decayavg(float a, float in, float w)  
{ return a + (in - a) / w; }  
  
// rtty.cxx:453-465  старт-бит: mark в старом конце буфера + space в новом  
static bool rtty_is_mark_space(int &correction)  
{  
    correction = 0;  
    if (rrx.bit_buf[0] && !rrx.bit_buf[rrx.symbollen - 1]) {  
        for (int i = 0; i < rrx.symbollen; i++)  
            correction += rrx.bit_buf[i];  
        if (abs(rrx.symbollen / 2 - correction) < 6)  
            return true;  
    }  
    return false;  
}  
  
static inline bool rtty_is_mark(void)  
{ return rrx.bit_buf[rrx.symbollen / 2]; }  
  
// rtty.cxx:472-540  автомат состояний — копия  
static bool rtty_rx(bool bit)  
{  
    bool flag = false;  
    int correction;  
  
    for (int i = 1; i < rrx.symbollen; i++) rrx.bit_buf[i-1] = rrx.bit_buf[i];  
    rrx.bit_buf[rrx.symbollen - 1] = bit;  
  
    switch (rrx.rxstate) {  
    case 0: // IDLE  
        if (rtty_is_mark_space(correction)) {  
            rrx.rxstate = 1; rrx.counter = correction;  
        }  
        break;  
    case 1: // START  
        if (--rrx.counter == 0) {  
            if (!rtty_is_mark()) {  
                rrx.rxstate = 2; rrx.counter = rrx.symbollen;  
                rrx.bitcntr = 0; rrx.rxdata = 0;  
            } else rrx.rxstate = 0;  
        }  
        break;  
    case 2: // DATA  
        if (--rrx.counter == 0) {  
            rrx.rxdata |= (rtty_is_mark() ? 1 : 0) << rrx.bitcntr++;  
            rrx.counter = rrx.symbollen;  
        }  
        if (rrx.bitcntr == rrx.nbits) rrx.rxstate = 3;  
        break;  
    case 3: // STOP  
        if (--rrx.counter == 0) {  
            if (rtty_is_mark()) {  
                uint8_t c = (uint8_t)(rrx.rxdata & 0x1F);  
                rtty_emit(c);  
                flag = true;  
            }  
            rrx.rxstate = 0;  
        }  
        break;  
    }  
    return flag;  
}

static void rtty_rx_sample(float xr, float xq)  
{  
    // boxcar-обновления: выталкиваем старое, кладём новое  
    float m_r, m_i, s_r, s_i;  
    rtty_mixer(rrx.mark_phase,  rrx.f_mark,  xr, xq, m_r, m_i);  
    rtty_mixer(rrx.space_phase, rrx.f_space, xr, xq, s_r, s_i);  
  
    int L = rrx.symbollen, idx = rrx.buf_idx;  
    rrx.sum_mi += m_r - rrx.buf_mi[idx]; rrx.buf_mi[idx] = m_r;  
    rrx.sum_mq += m_i - rrx.buf_mq[idx]; rrx.buf_mq[idx] = m_i;  
    rrx.sum_si += s_r - rrx.buf_si[idx]; rrx.buf_si[idx] = s_r;    
    rrx.sum_sq += s_i - rrx.buf_sq[idx]; rrx.buf_sq[idx] = s_i;
    if (++rrx.buf_idx >= L) rrx.buf_idx = 0;  
  
    rrx.mark_mag  = sqrtf(rrx.sum_mi*rrx.sum_mi + rrx.sum_mq*rrx.sum_mq) / L;  
    rrx.space_mag = sqrtf(rrx.sum_si*rrx.sum_si + rrx.sum_sq*rrx.sum_sq) / L;  
  
    // rtty.cxx:679-693 — асимметричные decayavg  
    float T = rrx.symbollen_f;  
    rrx.mark_env   = rtty_decayavg(rrx.mark_env,  rrx.mark_mag,  
                     rrx.mark_mag  > rrx.mark_env  ? T/4  : T*16);  
    rrx.mark_noise = rtty_decayavg(rrx.mark_noise, rrx.mark_mag,  
                     rrx.mark_mag  < rrx.mark_noise ? T/4  : T*48);  
    rrx.space_env  = rtty_decayavg(rrx.space_env, rrx.space_mag,  
                     rrx.space_mag > rrx.space_env ? T/4  : T*16);  
    rrx.space_noise= rtty_decayavg(rrx.space_noise,rrx.space_mag,  
                     rrx.space_mag < rrx.space_noise? T/4  : T*48);  
    rrx.noise_floor = fminf(rrx.space_noise, rrx.mark_noise);  
  
    // clipped ATC + optimal v3 — rtty.cxx:696-725  
    float mclipped = rrx.mark_mag  > rrx.mark_env  ? rrx.mark_env  : rrx.mark_mag;  
    float sclipped = rrx.space_mag > rrx.space_env ? rrx.space_env : rrx.space_mag;  
    if (mclipped < rrx.noise_floor) mclipped = rrx.noise_floor;  
    if (sclipped < rrx.noise_floor) sclipped = rrx.noise_floor;  
  
    float mn = rrx.mark_env  - rrx.noise_floor;  
    float sn = rrx.space_env - rrx.noise_floor;  
    float v3 = (mclipped - rrx.noise_floor) * mn  
             - (sclipped - rrx.noise_floor) * sn  
             - 0.25f * (mn*mn - sn*sn);  
  
    bool bit = (v3 > 0);  

    rrx.sym_since_char++;                        // считаем отсчёты между символами  
    if (rtty_rx(bit)) {    
        // поворот суммы mark-канала за интервал между двумя символами    
        float dphi = atan2f(rrx.h_m_re[1]*rrx.h_m_im[0] - rrx.h_m_im[1]*rrx.h_m_re[0],    
                            rrx.h_m_re[1]*rrx.h_m_re[0] + rrx.h_m_im[1]*rrx.h_m_im[0]);    
        float ferr = 0;    
        if (rrx.sym_since_char > 0)    
            ferr = dphi * rrx.fs / (2.0f * (float)M_PI * (float)rrx.sym_since_char);    
        if (fabsf(ferr) > 22.7f) ferr = 0;       // fldigi: clamp shift/8    
        rrx.freqerr = rtty_decayavg(rrx.freqerr, ferr, 8);    
        rrx.f_mark  -= rrx.freqerr;    
        rrx.f_space -= rrx.freqerr;    
        rrx.sym_since_char = 0;    
        // сдвиг истории: [0] = сумма прошлого символа  
        rrx.h_m_re[0] = rrx.h_m_re[1]; rrx.h_m_im[0] = rrx.h_m_im[1];  
        rrx.h_s_re[0] = rrx.h_s_re[1]; rrx.h_s_im[0] = rrx.h_s_im[1];  
        // [1] = сумма текущего символа (сумматоры живут до следующего бита)  
        rrx.h_m_re[1] = rrx.sum_mi;  rrx.h_m_im[1] = rrx.sum_mq;  
        rrx.h_s_re[1] = rrx.sum_si;  rrx.h_s_im[1] = rrx.sum_sq;
    }  
}

// ============================================================================    
// 1. GOERTZEL-ДЕТЕКТОР (комплексный банк тонов)    
// ============================================================================    
// Однополюсный комплексный резонатор s = x + c·s, c = e^{jw}, Q15.  
// gtz_power = |s|^2 >> 10 с сатурацией в INT32_MAX.  
// gtz_reset обязателен после каждого окна — без него ar/ai растут безгранично.  
    
#define DEC_MAX_TONES 33         // максимум тонов (FSQ/IFKP = 33, JTTY = 4)  
    
typedef struct {    
    int32_t cr, ci;        // e^{jω} в Q15: cr=cos, ci=sin    
    int32_t ar, ai;        // комплексный резонатор s = z + c·s    
} gtz_t;  
    
static gtz_t  gtz[DEC_MAX_TONES]; // банк фильтров    
static uint8_t gtz_ntones = 0;    // сколько тонов реально активно    
    
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
    
static inline int32_t gtz_power(uint8_t t) {    
    const gtz_t* g = &gtz[t];    
    int64_t p = (int64_t)g->ar * g->ar + (int64_t)g->ai * g->ai;    
    p >>= 10;                                     // масштаб под int32    
    if (p > 0x7FFFFFFFLL) p = 0x7FFFFFFFLL;    
    return (int32_t)p;    
}  
    
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
  
static void gtz_bank_init(float f0_hz, float step_hz, uint8_t ntones, float fs) {    
    gtz_ntones = (ntones > DEC_MAX_TONES) ? DEC_MAX_TONES : ntones;    
    for (uint8_t t = 0; t < gtz_ntones; t++) {    
        float w = 2.0f * PI * (f0_hz + t * step_hz) / fs;    
        gtz[t].cr = (int32_t)(32768.0f * cosf(w));    
        gtz[t].ci = (int32_t)(32768.0f * sinf(w));    
        gtz_reset(&gtz[t]);    
    }    
}  
  
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
    uint8_t  sym_hist[5];  
    uint8_t  hist_n;    
    uint8_t  walsh_idx;  
    int      walsh_accum[8];    // накопление корреляций для каждой из 8 строк    
    uint8_t  block_pos;         // 0..63 — позиция символа в блоке (64 символа)    
    uint8_t  block_bits[64];    // по 3 бита с каждого символа → поток битов    
} olivia_dec_t;    
static olivia_dec_t olv;    
    
static void olivia_on_tone(uint8_t tone) {    
    uint8_t sym = olivia_tone2sym[tone & 7];    
    for (int k = 0; k < 8; k++)    
        olv.walsh_accum[k] += walsh8[k][sym];    
    if (++olv.block_pos == 64) {    
        // TODO: полный путь — дескремблер + сортировка блоков (olivia.cxx)    
        memset(olv.walsh_accum, 0, sizeof(olv.walsh_accum));    
        olv.block_pos = 0;    
    }    
}    
  
// ============================================================================    
// 3. RTTY — порт ядра fldigi rtty.cxx    
// ============================================================================    
// Цепочка на отсчёт:    
//   x -> mix(mark_hz) -> boxcar(T бит) -> env_m          -> ATC v3 -> бит-таймер    
//   x -> mix(space_hz)-> boxcar(T бит) -> env_s -> v1,v2,v3    
//    
// Почему это решает проблему расстройки ±100+ Гц: миксер отслеживает частоту    
// через AFC (остаточное вращение фильтра за бит), а НЧ-фильтр шириной ~1/T    
// пропускает тон даже при сдвиге в ±30-40 Гц — за это время AFC сходится.  
  
// decayavg из fldigi misc.h — экспоненциальное среднее с весом w    
static inline float decayavg(float avg, float input, float w) {    
    if (w <= 1.0f) return input;    
    return avg + (input - avg) / w;    
}  
  
static const char baudot_ltrs[32] = {    
    '\0','E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C','K',    
    'T','Z','L','W','H','Y','P','Q','O','B','G',0,'M','X','V',0 };    
static const char baudot_figs[32] = {    
    '\0','3','\n','-',' ','\'','8','7','\r','$','4','\a',',','!',':','(',    
    '5','+',')','2','#','6','0','1','9','?','&',0,'.','/',';',0 };    
// МТК-2 русский регистр: UTF-8 строки (кириллица = 2 байта, char не вмещает)    
static const char *baudot_rus[32] = {    
    "",  "Е", "\n", "А", " ", "С", "И", "У",    
    "\r","Д", "Р", "Й", "Н", "Ф", "Ц", "К",    
    "Т", "З", "Л", "В", "Х", "Ы", "П", "Я",    
    "О", "Б", "Г", "",  "М", "Ь", "Ш", "" };  
  
#define MTK2_RUS  0x00    
#define MTK2_LAT  0x1F    
#define MTK2_FIG  0x1B  
  



  
// Выдать собранный символ Бодо/МТК-2  
static void rtty_emit(uint8_t c) {  
    if (dbg_rx)  
        Serial.printf("[RTTY] shreg=0x%02x figs=%u err=%.1f\n",  
                      (unsigned)c, (unsigned)rrx.figs, (double)rrx.freqerr);
    if (c == MTK2_FIG)      { rrx.figs = 1; return; }  
    if (c == MTK2_LAT)      { rrx.figs = 0; return; }  
    // 0x00 = blank (ITA-2), НЕ переключаем регистр: иначе каждый blank/  
    // сбитый символ уводит декодер в RUS и латиница печатается кириллицей.  
    if (c == 0x00) return;  
  
    if (rrx.figs == 2) {  
        const char *s = baudot_rus[c];  
        if (s[0]) Serial.print(s);  
    } else {  
        char ch = rrx.figs ? baudot_figs[c] : baudot_ltrs[c];  
        if (ch) Serial.write(ch);  
    }
}  

  

  
// ----------------------------------------------------------------------------    
// 4. CW ДЕКОДЕР: фронты -> элементы -> знаки (адаптивная классификация UHSDR)    
// ----------------------------------------------------------------------------    
    
// Бинарное дерево Морзе: '.'→2k+1, '-'→2k+2, корень idx=0.    
// Глубина до 6 элементов → узлов 2^7-1 = 127    
static char cw_tree[128];  
    
static void cw_tree_init(void) {    
    memset(cw_tree, ' ', sizeof(cw_tree));    
    static const char *tab[][2] = {    
        {"A",".-"},{"B","-..."},{"C","-.-."},{"D","-.."},{"E","."},    
        {"F","..-."},{"G","--."},{"H","...."},{"I",".."},{"J",".---"},    
        {"K","-.-"},{"L",".-.."},{"M","--"},{"N","-."},{"O","---"},    
        {"P",".--."},{"Q","--.-"},{"R",".-."},{"S","..."},{"T","-"},    
        {"U","..-"},{"V","...-"},{"W",".--"},{"X","-..-"},{"Y","-.--"},    
        {"Z","--.."},    
        {"1",".----"},{"2","..---"},{"3","...--"},{"4","....-"},    
        {"5","....."},{"6","-...."},{"7","--..."},{"8","---.."},    
        {"9","----."},{"0","-----"},    
        {"/","-..-."},{",","--..--"},{".",".-.-.-"},{"?","..--.."},    
        {"=","-...-"},{"'",".----."},{"-","-....-"},{"(","-.--."},    
        {")","-.--.-"},{":","---..."},{";","-.-.-."},{"+",".-.-."},    
        {"\"",".-..-."},{"@",".--.-."},    
    };    
    for (unsigned i = 0; i < sizeof(tab)/sizeof(tab[0]); i++) {    
        int idx = 0;    
        for (const char *s = tab[i][1]; *s && idx < 127; s++)    
            idx = idx * 2 + (*s == '-' ? 2 : 1);    
        if (idx < 128) cw_tree[idx] = tab[i][0][0];    
    }  
}  
    
static void cw_decode_elements(void) {    
    if (!cw.elem_n) return;    
    cw.elem_buf[cw.elem_n] = 0;    
    int idx = 0;    
    for (uint8_t i = 0; i < cw.elem_n && idx < 127; i++)    
        idx = idx * 2 + (cw.elem_buf[i] == '-' ? 2 : 1);    
    char c = (idx < 128) ? cw_tree[idx] : '?';  
    if (dbg_rx)    
        Serial.printf("\n[ELEMS] '%s' idx=%d -> '%c'\n", cw.elem_buf, idx, c ? c : '?');    
    if (c && c != ' ') Serial.write(c);    
    cw.elem_n = 0;    
}  
    
// on=true  -> завершилась пауза (фронт нажатия)    
// on=false -> завершился тон (фронт отпускания)    
// idx — sample_idx текущего блока, fs_x1000 = fs_out*1000    
static void cw_on_edge(bool on, uint32_t idx, uint32_t fs_x1000) {    
    uint32_t dur_smp = on ? (idx - cw.off_start) : (idx - cw.on_start);    
    float dur_ms = (float)dur_smp * 1.0e6f / (float)fs_x1000;    
    
    if (dbg_rx)    
        Serial.printf("[EDGE] %s dur=%.1fms dot=%.1f\n",    
                      on ? "on " : "off", (double)dur_ms, (double)cw_dot_avg);    
    
    if (on) {    
        // ---------------- ЗАВЕРШИЛАСЬ ПАУЗА ----------------    
        if (dur_ms < 20.0f) return;                          // микрощелчок    
    
        if      (dur_ms > cw_dot_avg * 4.5f) { cw_decode_elements(); Serial.write(' '); }    
        else if (dur_ms > cw_dot_avg * 2.0f) { cw_decode_elements(); }    
    
        // Буфер насытился без паузы-разделителя (длинная помеха) — сброс мусора    
        if (cw.elem_n >= sizeof(cw.elem_buf) - 1) {    
            if (dbg_rx) Serial.println(F("[CW] elem_buf satur — сброс"));    
            cw.elem_n = 0;    
        }    
    
        if (dur_ms < cw_dot_avg * 2.0f)    
            cw_gap_avg += (dur_ms - cw_gap_avg) * 0.1f;    
    
    } else {    
        // ---------------- ЗАВЕРШИЛСЯ ИМПУЛЬС ----------------    
        if (dur_ms < 20.0f) return;                          // статический разряд    
    
        // Залипание > 2 с (помеха, перегрузка AGC) — не элемент Морзе    
        if (dur_ms > 2000.0f) { cw.elem_n = 0; return; }    
    
        float pulse_avg = (cw_dot_avg + cw_dash_avg) * 0.5f;    
        char elem;    
    
        if (dur_ms < pulse_avg) {    
            elem = '.';    
            if (dur_ms > 25.0f && dur_ms < cw_dot_avg * 1.8f)    
                cw_dot_avg += (dur_ms - cw_dot_avg) * 0.15f;    
        } else {    
            elem = '-';    
            if (dur_ms > cw_dot_avg * 2.0f && dur_ms < cw_dash_avg * 1.8f)    
                cw_dash_avg += (dur_ms - cw_dash_avg) * 0.05f;    
        }    
    
        // Удержание соотношения тире:точка ~3:1    
        if (cw_dash_avg < cw_dot_avg * 2.5f) cw_dash_avg = cw_dot_avg * 3.0f;    
        if (cw_dash_avg > cw_dot_avg * 4.0f) cw_dash_avg = cw_dot_avg * 3.0f;    
    
        if (cw.elem_n < sizeof(cw.elem_buf) - 1)    
            cw.elem_buf[cw.elem_n++] = elem;    
    
        if (dbg_rx)    
            Serial.printf("[ELEM] '%c' dur=%.1fms thr=%.1f dot=%.1f dash=%.1f\n",    
                          elem, (double)dur_ms, (double)pulse_avg,    
                          (double)cw_dot_avg, (double)cw_dash_avg);    
    }    
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
// rx_dsp вращает спектр так, что center_hz (RF) → 0 Гц.    
void rx_decoders_start(uint8_t mode, float center_hz) {  
    if (dbg_rx) {Serial.printf("[DEC] start: mode=%u center=%.1f\n",    
                (unsigned)mode, (double)center_hz);}  
    dec_mode = (rx_mode_t)mode;    
    fs_out = rx_dsp_fs_out();    
    if (dbg_rx) {Serial.printf("[DEC] fs_out=%.1f\n", (double)fs_out);}  
    f_center_hz = center_hz;    
    sym_acc = 0; sample_idx = 0;    
    memset(&cw, 0, sizeof(cw));    
    memset(&olv, 0, sizeof(olv)); memset(&ifk, 0, sizeof(ifk));  
    // Гибридный CW: сброс детектора огибающей и адаптивных оценок таймингов    
    cw_p_ema     = 0;    
    cw_low_cnt   = 0;    
    cw_sig_avg   = 0;         // детектор сам выйдет на уровень за ~16 блоков    
    cw_noise_avg = 1024;      // стартовый шум; если ключ «залипнет» — уменьшить до ~256    
    cw_dot_avg   = 60.0f;     // 60 мс ≈ 20 WPM    
    cw_dash_avg  = 180.0f;    
    cw_gap_avg   = 60.0f;    // пауза между элементами ≈ 1 точка  
    memset(jtty_hist, 0xFF, sizeof(jtty_hist));   // гарантированно не совпадёт с is13    
    cw.tone_on  = false;    
    cw.on_start = cw.off_start = 0;    
    cw.elem_n   = 0;
    jtty_state = 0; jtty_symcnt = 0;
      
    cw_tree_init();  
  
    switch (dec_mode) {    
    case RX_MODE_CW:    
        // Тон реально сидит ~+63 Гц выше базовой полосы — остаточный сдвиг LO    
        gtz_bank_init(63.0f, 0.0f, 1, fs_out);    
        sym_samples = (uint32_t)(fs_out * 0.005f);    
        Serial.printf("[CW-RX] bank ready, gtz_ntones=%u sym=%lu\n",    
                      gtz_ntones, (unsigned long)sym_samples);    
        break;  
    case RX_MODE_RTTY:  
        memset(&rrx, 0, sizeof(rrx));  
        rrx.fs        = fs_out;  
        rrx.symbollen_f = fs_out / 45.45f;  
        rrx.symbollen   = (int)(rrx.symbollen_f + 0.5f);   // 344  
        rrx.nbits     = 5;  
        rrx.stoplen   = (int)(1.5f * rrx.symbollen_f + 0.5f);  
        rrx.f_mark    = -85.0f;    // тона измерены [SCAN]: -85 и +85  
        rrx.f_space   = +85.0f;    // при зеркальном тексте поменять местами  
        rrx.figs      = 0;  
        rrx.lastchar  = 0;  
        break;

        break;
    case RX_MODE_OLIVIA:    
    case RX_MODE_CONTESTIA:    
        gtz_bank_init(-109.375f, 31.25f, 8, fs_out);    
        sym_samples = (uint32_t)(fs_out * 0.032f);   // 32 мс    
        break;    
    case RX_MODE_MFSK16:    
        gtz_bank_init(-117.19f, 15.625f, 16, fs_out);    
        sym_samples = (uint32_t)(fs_out * 0.064f);   // 64 мс    
        break;    
    case RX_MODE_THOR:    
        gtz_bank_init(-132.81f, 15.625f, 18, fs_out);    
        sym_samples = (uint32_t)(fs_out * 0.128f);   // 128 мс (THOR8)    
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
        gtz_bank_init(-46.875f, 31.25f, 4, fs_out);  // тоны -47,-16,+16,+47 Гц    
        sym_samples = (uint32_t)(fs_out * 0.032f);   // 500 отсчётов = 32 мс    
        Serial.printf("[JTTY-RX] baseband tones -46.9..+46.9 Hz step=31.25 sym=%lu smp fs=%.1f\n",    
                      (unsigned long)sym_samples, (double)fs_out);    
        break;    
    default: break;    
    }    
}  
    
// AFC для мод на банке Гёрцеля — заглушка, будет дописана отдельно    
static void afc_update(uint8_t tone, int32_t pwr) {  
    (void)tone; (void)pwr;  
    if (!afc_on || gtz_ntones < 2) return;    
}

static inline void jtty_push(uint8_t dibit) {  
    if (jtty_symcnt < sizeof(jtty_sym))  
        jtty_sym[jtty_symcnt] = dibit;  
    jtty_symcnt++;          // счётчик растёт, запись — только в пределах буфера  
    if (jtty_symcnt >= 78) {  
        Serial.print(F("[JTTY] frame:"));  
        for (uint8_t i = 13; i < 59 && i < sizeof(jtty_sym); i++)  
            Serial.printf("%d", jtty_sym[i]);  
        Serial.println();  
        jtty_symcnt = 0; jtty_inframe = 0; jtty_state = 0;  
    }  
}
  
void rx_decoders_stop(void) { dec_mode = RX_MODE_NONE; }    
    
// Вызывается из цикла core1 (rx_dsp_entry) после заполнения out_i/out_q    
void rx_decoders_process(const int16_t* xi, const int16_t* xq, uint8_t n) {  
    static uint32_t calls = 0;
    if (++calls == 1 || calls % 1000 == 0)    
        if (dbg_rx) {Serial.printf("[DEC] process: calls=%lu mode=%u n=%u\n",    
                      (unsigned long)calls, (unsigned)dec_mode, (unsigned)n);}  
  
    if (dec_mode == RX_MODE_NONE || n == 0) return;  
  
// ---------------- CW: блок = одно окно детектора огибающей ----------------    
    if (dec_mode == RX_MODE_CW) {    
        gtz_push(xi, xq, n);    
        int32_t p; gtz_winner(&p);    
    
        // EMA огибающей ~8 окон (~33 мс): провалы внутри тире не рвут посылку    
        cw_p_ema += (p - cw_p_ema) >> 3;    
        int32_t pe = cw_p_ema;    
    
        // Двухуровневая AGC + гистерезис (fldigi):    
        if (pe > cw_noise_avg)    
            cw_noise_avg += (pe - cw_noise_avg) >> 8;   // очень медленно вверх    
        else    
            cw_noise_avg += (pe - cw_noise_avg) >> 3;   // быстро вниз    
    
        if (cw.tone_on && pe > cw_sig_avg) cw_sig_avg += (pe - cw_sig_avg) >> 1;    
        else                               cw_sig_avg -= (cw_sig_avg >> 8);   // ~2%/окно    
        if (!cw.tone_on)    
            cw_noise_avg += (pe - cw_noise_avg) >> 4;                // floor — в паузах    
        int32_t range = cw_sig_avg - cw_noise_avg; if (range < 1) range = 1;    
        int32_t thr_hi = cw_noise_avg + (range * 3) / 5;    
        int32_t thr_lo = cw_noise_avg + (range * 2) / 5;  
  
        bool key;    
        if      (pe > thr_hi) key = true;    
        else if (pe < thr_lo) key = false;    
        else                  key = cw.tone_on;         // зона гистерезиса    
    
        // Дебаунс: off только после 3 подряд «тихих» окон (~12 мс)    
        bool key_deb;    
        if (key) { cw_low_cnt = 0; key_deb = true; }    
        else {    
            if (cw_low_cnt < 3) { cw_low_cnt++; key_deb = cw.tone_on; }    
            else                  key_deb = false;    
        }    
    
        // Отладка: огибающая раз в ~320 мс    
        static uint32_t dbg = 0;    
        if (++dbg >= 64) {    
            dbg = 0;    
            if (dbg_rx)    
                Serial.printf("[CW] pe=%ld sig=%ld nse=%ld %s dot=%.1fms\n",    
                              (long)pe, (long)cw_sig_avg, (long)cw_noise_avg,    
                              key_deb ? "ON" : "off", (double)cw_dot_avg);    
        }    
    
        if (key_deb != cw.tone_on) {    
            cw_on_edge(key_deb, sample_idx, (uint32_t)(fs_out * 1000.0f));    
            cw.tone_on = key_deb;    
            if (key_deb) cw.on_start = sample_idx; else cw.off_start = sample_idx;    
        }  
        for (uint8_t t = 0; t < gtz_ntones; t++) gtz_reset(&gtz[t]);    
        sample_idx += n;    
        return;    
    }  
  
// ------------- RTTY: поотсчётный тракт fldigi, окна символов не нужны -------  
if (dec_mode == RX_MODE_RTTY) {  
        for (uint8_t i = 0; i < n; i++)  
            rtty_rx_sample((float)xi[i], (float)xq[i]);  
        return;  
    }
    
    // ------------- Остальные моды: символьная синхронизация по блокам ----------    
    uint8_t pos = 0;    
    while (pos < n) {    
        uint32_t remain_sym = sym_samples - sym_acc;    
        uint8_t  chunk      = (uint8_t)((uint32_t)(n - pos) < remain_sym    
                                        ? (n - pos) : remain_sym);    
    
        gtz_push(xi + pos, xq + pos, chunk);   // комплексный вход: I и Q  
    
        sym_acc    += chunk;    
        pos        += chunk;    
        sample_idx += chunk;    
    
        if (sym_acc < sym_samples) break;        // окно не закрыто — выходим    
    
        // Граница символа: winner-take-all    
        int32_t pmax;    
        uint8_t tone = gtz_argmax(&pmax);  
        afc_update(tone, pmax);    
    
        switch (dec_mode) {  
        case RX_MODE_OLIVIA: case RX_MODE_CONTESTIA:    
            olivia_on_tone(tone);    
            break;    
        case RX_MODE_MFSK16: {    
            uint8_t nib = tone;               // gray → bin    
            nib ^= nib >> 2; nib ^= nib >> 1;    
            // TODO: склейщик битов → varicode[]    
            (void)nib;    
            break;    
        }    
        case RX_MODE_THOR: case RX_MODE_DOMEX:    
        case RX_MODE_FSQ:  case RX_MODE_IFKP:    
            ifk_on_tone(tone);                // TODO: varicode/FEC    
            break;  
        case RX_MODE_JTTY: {    
            // Отладка: статистика тонов раз в ~0.5 с    
            static int32_t  j_acc[4]; static uint8_t j_n = 0;    
            for (uint8_t t = 0; t < 4 && t < gtz_ntones; t++)    
                j_acc[t] += gtz_power(t);    
            if (++j_n >= 16) {      
                if (dbg_rx)    
                    Serial.printf("[JTTY] pwr: %ld %ld %ld %ld  win=%d\n",      
                                  (long)j_acc[0]/j_n, (long)j_acc[1]/j_n,      
                                  (long)j_acc[2]/j_n, (long)j_acc[3]/j_n, tone);      
                j_acc[0]=j_acc[1]=j_acc[2]=j_acc[3]=0; j_n=0;  
            }  
  
            // jtty_sync: кадр = 78 дибитов; символы 13..58 — данные TBCC  
            switch (jtty_state) {  
            case 0:                                     // ждём тон 0  
                if (tone == 0) { jtty_state = 1; jtty_symcnt = 0; jtty_inframe = 1; }  
                break;  
            case 1:                                     // ждём тон 1  
                if (tone == 1) { jtty_state = 2; }  
                else if (tone != 0) {  
                    jtty_push(1);  
                }  
                break;  
case 2:                                     // в кадре: собираем 78 дибитов  
                if (jtty_inframe) {  
                    jtty_push(tone);  
                } else if (tone == 1) {  
                    jtty_push(1);  
                }  
                break;  
            default:  
                jtty_state = 0; jtty_inframe = 0;  
                break;  
            }                                           // конец switch (jtty_state)  
            break;                                      // конец case RX_MODE_JTTY  
        }  
  
        default: break;  
        }                                               // конец switch (dec_mode)  
  
        // Сброс резонаторов — для всех символьных режимов.  
        // gtz — однополюсный ИНТЕГРАТОР (s = x + c·s): без сброса ar/ai  
        // растут бесконечно → gtz_power насыщается в INT32_MAX.  
        // (CW и RTTY выше выходят через return и сюда не доходят.)  
        for (uint8_t t = 0; t < gtz_ntones; t++) gtz_reset(&gtz[t]);  
        sym_acc   = 0;  
    }                                                   // конец while (pos < n)  
}                                                       // конец rx_decoders_process

