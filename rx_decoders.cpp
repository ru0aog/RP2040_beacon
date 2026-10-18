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
  
// Длина скользящего среднего = отсчётов на бит (344 при 45.45 бод, 15.6 кс/с).  
// Запас 512 покрывает бодрейты до ~30 бод при fs_out=15.6к.  
#define RTTY_MA_LEN 512  
  
typedef struct {  
    // --- миксеры (NCO на тон) ---  
    float    ph_m_re, ph_m_im;    // фазор миксера mark  
    float    ph_s_re, ph_s_im;    // фазор миксера space  
    float    stp_m_re, stp_m_im;  // шаг вращения mark  = e^{j 2π f_m/fs}  
    float    stp_s_re, stp_s_im;  // шаг вращения space  
    float    f_m_hz, f_s_hz;      // текущие частоты миксеров (AFC их двигает)  
    float    f_m_nom, f_s_nom;    // номиналы (для ограничения AFC)  
  
    // --- boxcar согласованные фильтры (комплексные суммы) ---  
    float    buf_mr[RTTY_MA_LEN], buf_mi[RTTY_MA_LEN];  
    float    buf_sr[RTTY_MA_LEN], buf_si[RTTY_MA_LEN];  
    float    sum_mr, sum_mi, sum_sr, sum_si;  
    uint16_t ma_pos;              // позиция кольца  
    uint16_t ma_len;              // = отсчётов на бит  
    uint16_t ma_fill;             // сколько отсчётов реально набрано  
  
    // --- детекторы огибающей / ATC (fldigi) ---  
    float    mark_env, space_env;  
    float    noise_floor;  
    float    v1, v2, v3;          // тройное сглаживание разности  
  
    // --- предыдущий выход согласованного фильтра (для AFC) ---  
    float    prev_mac_m_re, prev_mac_m_im;  
    float    prev_mac_s_re, prev_mac_s_im;  
    float    freqerr;             // Гц, оценка расстройки (EMA)  
  
    // --- битовый таймер ---  
    uint8_t  state;               // 0=idle(mark), 1=датабиты, 2=стоп  
    uint8_t  bit_cnt;  
    uint8_t  shreg;  
    uint8_t  figs;                // 0=LTRS, 1=FIGS, 2=RUS  
    uint16_t bit_timer;           // отсчётов до следующего решения  
    uint16_t bit_period;          // отсчётов на бит  
    bool     lastbit;             // предыдущее решение (фронт mark→space)  
    uint8_t  space_cnt;           // деглитчинг старт-бита  
    uint32_t renorm_cnt;          // счётчик для ренормировки фазоров  
} rtty_rx_t;  
  
static rtty_rx_t rrx;  
  
// Инициализация миксера: частота f_hz в базовой полосе, фазор шага = e^{j w}  
static inline void rtty_set_tone(float* sre, float* sim, float f_hz, float fs) {  
    float w = 2.0f * PI * f_hz / fs;  
    *sre = cosf(w);  
    *sim = sinf(w);  
}  
  
static void rtty_rx_init(float mark_hz, float space_hz, float baud, float fs) {    
    // Безопасное явное обнуление скалярных полей структуры rtty_rx_t  
    rrx.ma_pos    = 0;  
    rrx.ma_fill   = 0;  
    rrx.state     = 0;  
    rrx.bit_cnt   = 0;  
    rrx.shreg     = 0;  
    rrx.space_cnt = 0;  
    rrx.renorm_cnt = 0;  
    rrx.figs      = 0;  // Стартовая раскладка: LAT (латиница)
    rrx.bit_timer = 0;  
    
    rrx.sum_mr = rrx.sum_mi = rrx.sum_sr = rrx.sum_si = 0.0f;  
    rrx.mark_env = rrx.space_env = rrx.v1 = rrx.v2 = rrx.v3 = 0.0f;  
    rrx.prev_mac_m_re = rrx.prev_mac_m_im = rrx.prev_mac_s_re = rrx.prev_mac_s_im = 0.0f;  
    rrx.freqerr   = 0.0f;  
  
    // Явное покомпонентное обнуление массивов скользящего Boxcar-фильтра
    // Исключает невыровненные ассемблерные вставки memset больших структур в ARM-GCC
    for (uint16_t i = 0; i < RTTY_MA_LEN; i++) {  
        rrx.buf_mr[i] = 0.0f; rrx.buf_mi[i] = 0.0f;  
        rrx.buf_sr[i] = 0.0f; rrx.buf_si[i] = 0.0f;  
    }  
  
    // Запись номиналов частот и инициализация шагов вращения фазоров
    rrx.f_m_hz  = rrx.f_m_nom = mark_hz;  
    rrx.f_s_hz  = rrx.f_s_nom = space_hz;  
    rtty_set_tone(&rrx.stp_m_re, &rrx.stp_m_im, mark_hz,  fs);  
    rtty_set_tone(&rrx.stp_s_re, &rrx.stp_s_im, space_hz, fs);  
    rrx.ph_m_re = rrx.ph_s_re = 1.0f;  
  
    rrx.bit_period = (uint16_t)(fs / baud + 0.5f);  
    rrx.ma_len     = rrx.bit_period;  
    if (rrx.ma_len > RTTY_MA_LEN) rrx.ma_len = RTTY_MA_LEN;  
    rrx.lastbit    = true;           
    rrx.noise_floor = 1.0f;  
  
    Serial.printf("[RTTY-RX] fldigi core: mark=%.0f space=%.0f baud=%.2f T=%u smp\n",  
                  (double)mark_hz, (double)space_hz, (double)baud,  
                  (unsigned)rrx.bit_period);  
}


  
// Выдать собранный символ Бодо/МТК-2  
static void rtty_emit(uint8_t c) {  
    if (dbg_rx)  
        Serial.printf("[RTTY] shreg=0x%02x figs=%u err=%.1f\n",  
                      (unsigned)c, (unsigned)rrx.figs, (double)rrx.freqerr);  
    if      (c == MTK2_FIG) rrx.figs = 1;  
    else if (c == MTK2_LAT) rrx.figs = 0;  
    else if (c == MTK2_RUS) rrx.figs = 2;  
    else {  
        if (rrx.figs == 2) {  
            const char *s = baudot_rus[c];  
            if (s && s[0]) Serial.print(s);  
        } else {  
            char ch = rrx.figs ? baudot_figs[c] : baudot_ltrs[c];  
            if (ch) Serial.write(ch);  
        }  
    }  
}  


/**
 * Вычисление точной девиации частоты методом интерполяции Якобсена (Jacobsen IP).
 * Применяется для компенсации расстройки КВ-корреспондента на 100+ Гц.
 * Принимает текущие комплексные компоненты (реальная, мнимая) пика и его соседей.
 * Возвращает дробное смещение пика от центрального бина в диапазоне [-0.5 ... +0.5]
 */
// резерв: Jacobsen-интерполятор, нужен FFT-банк
static float rtty_interpolate_peak(float re_left,  float im_left,  
                                   float re_peak,  float im_peak,  
                                   float re_right, float im_right) {
    // Вычисляем вещественные компоненты кросс-произведений комплексных спектров
    // Математика: Real( (Y_{k-1} - Y_{k+1}) * conj(2*Y_k - Y_{k-1} - Y_{k+1}) )
    float num_re = (re_left - re_right) * (2.0f * re_peak - re_left - re_right) +
                   (im_left - im_right) * (2.0f * im_peak - im_left - im_right);

    float den_re = (2.0f * re_peak - re_left - re_right) * (2.0f * re_peak - re_left - re_right) +
                   (2.0f * im_peak - im_left - im_right) * (2.0f * im_peak - im_left - im_right);

    // Защита от деления на ноль при отсутствии сигнала
    if (den_re < 1.0e-5f) return 0.0f;

    float delta = num_re / den_re;

    // Жесткий защитный затвор: параболический сдвиг не может превышать половину бина
    if (delta >  0.5f) delta =  0.5f;
    if (delta < -0.5f) delta = -0.5f;

    return delta;
}



// AFC: остаточное вращение согласованного фильтра за один бит.  
// Если mac_winner повернулся на dphi за T бит → сигнал выше/ниже миксера на  
// err = dphi*fs/(2π*T). Поправка двигает оба миксера (один NCO на частоте).  
static void rtty_afc(void) {  
    bool m_win = (rrx.mark_env >= rrx.space_env);  
    float re  = m_win ? rrx.sum_mr : rrx.sum_sr;  
    float im  = m_win ? rrx.sum_mi : rrx.sum_si;  
    float pre = m_win ? rrx.prev_mac_m_re : rrx.prev_mac_s_re;  
    float pim = m_win ? rrx.prev_mac_m_im : rrx.prev_mac_s_im;  
  
    // Вычисляем квадраты модулей текущего и предыдущего векторов Boxcar-фильтра
    float m2_curr = re*re + im*im;  
    float m2_prev = pre*pre + pim*pim;
  
    // Защита от NaN-выбросов в паузах манипуляции
    if (m2_curr > 0.01f && m2_prev > 0.01f) {  
        // Кросс-произведение для вычисления остаточного вращения фазы за бит
        // ИСПРАВЛЕНО: Математически корректный знак комплексного дискриминатора fldigi.
        // Восстанавливает отрицательную обратную связь: петля AFC начинает притягивать 
        // частоты миксеров к истинному тону корреспондента, компенсируя расстройку до 250 Гц.
        float d = re * pre + im * pim;  
        float c = im * pre - re * pim;  

        
        // Быстрый замер фазового сдвига dphi = arg(z * conj(z_prev))
        float dphi = atan2f(c, d);  
        
        // Переводим радианы в герцы ошибки частоты: err = dphi * fs / (2 * PI * T_bit)
        float err_hz = dphi * fs_out / (2.0f * PI * (float)rrx.bit_period);  
        
        float total_err_hz = err_hz;   // интерполятору нужны FFT-бины, не boxcar-суммы

        // Жесткое ограничение: не более четверти сдвига частоты за один шаг (один бит)
        if (total_err_hz >  200.0f) total_err_hz =  200.0f;  
        if (total_err_hz < -200.0f) total_err_hz = -200.0f;  
        
        // Фильтрация ошибки через EMA (коэффициент затухания fldigi = 0.25)
        rrx.freqerr += (total_err_hz - rrx.freqerr) * 0.25f;  
        
        // Шаг подстройки петли (коэффициент жесткости = 0.2)
        float corr = rrx.freqerr * 0.2f;  
        float fm = rrx.f_m_hz + corr;  
        float fs = rrx.f_s_hz + corr;  
        
        // КВ-барьер безопасности: удерживаем миксеры в коридоре ±250 Гц от номинальной частоты тона
        fm = fminf(rrx.f_m_nom + 250.0f, fmaxf(rrx.f_m_nom - 250.0f, fm));  
        fs = fminf(rrx.f_s_nom + 250.0f, fmaxf(rrx.f_s_nom - 250.0f, fs));  
        
        rrx.f_m_hz = fm; rrx.f_s_hz = fs;  
        
        // На лету перестраиваем шаги вращения комплексных гетеродинов NCO
        rtty_set_tone(&rrx.stp_m_re, &rrx.stp_m_im, fm, fs_out);  
        rtty_set_tone(&rrx.stp_s_re, &rrx.stp_s_im, fs, fs_out);  
    }  
    
    // Сохраняем текущее комплексное состояние макроса для следующей итерации AFC
    rrx.prev_mac_m_re = rrx.sum_mr; rrx.prev_mac_m_im = rrx.sum_mi;  
    rrx.prev_mac_s_re = rrx.sum_sr; rrx.prev_mac_s_im = rrx.sum_si;  
}  


  
// Обработка одного отсчёта базовой полосы — порт rtty::rx_process  
static inline void rtty_rx_sample(int16_t xi, int16_t xq) {  
    // Нормализуем вход (масштабируем в 256 раз), защищая 32-битный float  
    // Boxcar-интегратор от знакового переполнения и улета в +Inf в блоке AFC
    float xr = (float)xi / 256.0f, xq_ = (float)xq / 256.0f;  
  
    // --- миксеры: перенос тонов в DC ---  
    float m_r = xr * rrx.ph_m_re + xq_ * rrx.ph_m_im;  
    float m_i = xq_ * rrx.ph_m_re - xr * rrx.ph_m_im;  
    float s_r = xr * rrx.ph_s_re + xq_ * rrx.ph_s_im;  
    float s_i = xq_ * rrx.ph_s_re - xr * rrx.ph_s_im;  
  
    // вращение фазоров  
    float t;  
    t = rrx.ph_m_re * rrx.stp_m_re - rrx.ph_m_im * rrx.stp_m_im;  
    rrx.ph_m_im = rrx.ph_m_re * rrx.stp_m_im + rrx.ph_m_im * rrx.stp_m_re;  
    rrx.ph_m_re = t;  
    t = rrx.ph_s_re * rrx.stp_s_re - rrx.ph_s_im * rrx.stp_s_im;  
    rrx.ph_s_im = rrx.ph_s_re * rrx.stp_s_im + rrx.ph_s_im * rrx.stp_s_re;  
    rrx.ph_s_re = t;  
  
    if (++rrx.renorm_cnt >= 2048) {  
        rrx.renorm_cnt = 0;  
        float n = 1.0f / sqrtf(rrx.ph_m_re*rrx.ph_m_re + rrx.ph_m_im*rrx.ph_m_im);  
        rrx.ph_m_re *= n; rrx.ph_m_im *= n;  
        n = 1.0f / sqrtf(rrx.ph_s_re*rrx.ph_s_re + rrx.ph_s_im*rrx.ph_s_im);  
        rrx.ph_s_re *= n; rrx.ph_s_im *= n;  
    }  
  
    // --- boxcar скользящее среднее ---  
    uint16_t p = rrx.ma_pos;  
    rrx.sum_mr += m_r - rrx.buf_mr[p]; rrx.buf_mr[p] = m_r;  
    rrx.sum_mi += m_i - rrx.buf_mi[p]; rrx.buf_mi[p] = m_i;  
    rrx.sum_sr += s_r - rrx.buf_sr[p]; rrx.buf_sr[p] = s_r;  
    rrx.sum_si += s_i - rrx.buf_si[p]; rrx.buf_si[p] = s_i;  
    if (++rrx.ma_pos >= rrx.ma_len) rrx.ma_pos = 0;  
    if (rrx.ma_fill < rrx.ma_len) rrx.ma_fill++;  
  
    // --- огибающие (Асимметричный decayavg fldigi) ---  
    float m_mag = sqrtf(rrx.sum_mr*rrx.sum_mr + rrx.sum_mi*rrx.sum_mi);  
    float s_mag = sqrtf(rrx.sum_sr*rrx.sum_sr + rrx.sum_si*rrx.sum_si);  
    
    // Временные константы атаки и спада fldigi, привязанные к длительности бита:
    // Атака (Fast Attack) = T_bit / 4 ≈ 85 отсчетов. Спад (Slow Decay) = T_bit * 16.
    float t_attack = (float)rrx.bit_period / 4.0f;
    float t_decay  = (float)rrx.bit_period * 16.0f;

    rrx.mark_env  = decayavg(rrx.mark_env,  m_mag, (m_mag > rrx.mark_env)  ? t_attack : t_decay);  
    rrx.space_env = decayavg(rrx.space_env, s_mag, (s_mag > rrx.space_env) ? t_attack : t_decay);  
    
    float lo = fminf(rrx.mark_env, rrx.space_env);  
    rrx.noise_floor = decayavg(rrx.noise_floor, lo, (float)rrx.bit_period * 48.0f);  
  
    // --- ATC v3 (fldigi) ---  
    float v = (rrx.mark_env - rrx.space_env) * 0.5f;  
    rrx.v1 = decayavg(rrx.v1, v,      16.0f);  
    rrx.v2 = decayavg(rrx.v2, rrx.v1, 32.0f);  
    rrx.v3 = decayavg(rrx.v3, rrx.v2, 64.0f);  
  
    // --- решение о бите ---  
    float sig_pk = fmaxf(rrx.mark_env, rrx.space_env);  
    bool  curbit;  
    if (rrx.ma_fill < rrx.ma_len || sig_pk < rrx.noise_floor * 1.5f)  
        curbit = true;  
    else  
        curbit = (rrx.v3 > 0.0f);  
  
    // --- битовый таймер (старт-стопный автомат) ---  
    switch (rrx.state) {  
    case 0:                                       // idle = mark, ловим фронт в space  
        if (!curbit) {  
            if (++rrx.space_cnt >= 4) {           // ~0.25 мс деглитчинг  
                rrx.state = 1;  
                rrx.bit_cnt = 0;  
                rrx.shreg = 0;  
                
                // ИСПРАВЛЕНО: Центрирование стробирования с учетом физики Boxcar-фильтра.
                // Групповая задержка скользящего среднего равна rrx.bit_period / 2.
                // Чтобы попасть строго в середину бита 0, первый дедлайн таймера должен составить
                // ровно половину периода бита (минус 4 отсчета деглитчера).
                rrx.bit_timer = rrx.bit_period + rrx.bit_period / 2 - 4;
                
                if (dbg_rx)  
                    Serial.println(F("[RTTY] start"));  
            }  
        } else {  
            rrx.space_cnt = 0;  
        }  
        break;  

  
    case 1:                                       
        if (--rrx.bit_timer == 0) {  
            if (curbit) rrx.shreg |= (1u << rrx.bit_cnt);   
            rtty_afc();                                   
            if (++rrx.bit_cnt >= 5) {  
                rrx.state = 2;  
                rrx.bit_timer = rrx.bit_period;           
            } else {  
                rrx.bit_timer = rrx.bit_period;  
            }  
        }  
        break;  
  
    case 2:                                       
        if (--rrx.bit_timer == 0) {  
            rrx.state = 0;  
            rrx.space_cnt = 0;  
            if (curbit) rtty_emit(rrx.shreg);  
            else if (dbg_rx) Serial.println(F("[RTTY] framing err"));  
        }  
        break;  
    }  
    rrx.lastbit = curbit;  
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
        // ДИАГНОСТИКА: 7 бинов через 85 Гц вокруг нуля — найти реальные тона  
        // gtz_bank_init(-256.0f, 85.33f, 7, fs_out);  
        // rtty_scan = true;  

        // Скан показал: реальные тоны на -85 и +85 Гц (bin2/bin4), сдвига LO нет.  
        // TX mark = нижний тон -> mark-миксер на -85, space-миксер на +85.  
        rtty_rx_init(-85.0f, 85.0f, 45.45f, fs_out);

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
        for (uint8_t i = 0; i < n; i++) rtty_rx_sample(xi[i], xq[i]);  
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

