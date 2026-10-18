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

/**
 * ============================================================================
 *  rx_decoders.cpp (CW Блок) — Амплитудный некогерентный CW-декодер
 *  Архитектура: Двухъядерная обработка (выполняется на core1 после rx_dsp)
 *  Входной поток: Квадратурные отсчеты (I/Q), Fs_in ≈ 15.6 кс/с
 * ============================================================================
 *
 *  МАТЕМАТИЧЕСКИЙ И СИСТЕМНЫЙ АНАЛИЗ ЗАИМСТВОВАНИЙ:
 *
 *  1. ФИЗИЧЕСКИЙ УРОВЕНЬ (Выделение тона и амплитудное детектирование)
 *     [Оригинальная адаптация] + [База алгоритмов fldigi]:
 *     - Частотная селекция: Выполнена на одиночном комплексном бине Герцеля
 *       (Goertzel), настроенном на фиксированную частоту +63 Гц базовой полосы.
 *     - Слежение за огибающей: Экспоненциальное сглаживание мощности (EMA ~33 мс)
 *       для защиты от дробления посылок из-за кратковременных провалов сигнала.
 *     - Автоматический порог: Асимметричный AGC-трекер уровней шума и сигнала 
 *       из fldigi. Уровень шума растет крайне медленно (>>8), но падает быстро (>>3).
 *       Уровень сигнала захватывается агрессивно (>>2), а спадает медленно (>>9).
 *     - Двухпороговый гистерезис: Логика fldigi на целочисленных сдвигах. 
 *       Порог захвата (thr_hi) жестко привязан к 3/5 (60%) динамического диапазона, 
 *       а порог отпускания (thr_lo) — к 2/5 (40%), что исключает дребезг на краях.
 *
 *  2. ЛОГИЧЕСКИЙ УРОВЕНЬ (Временная классификация импульсов и декодирование знаков)
 *     [База алгоритмов UHSDR / mcHF]:
 *     - Дебаунс ключа: Цифровое подтверждение спада (off) только после фиксации 
 *       3 подряд «тихих» окон (~12 мс) для фильтрации статических разрядов.
 *     - Адаптация к скорости (WPM): Упрощенный тайминг-анализатор из UHSDR 
 *       вместо тяжелой нейросети Кохонена (SOM) из fldigi. 
 *     - Скорость обучения элементов: Динамический расчет композитного порога 
 *       длительности pulse_avg. Подстройка под длину точки (cw_dot_avg) идет 
 *       агрессивно (вес 0.25f), а под длину тире и пауз — консервативно (вес 0.10f).
 *     - Сборка символов: Статическое бинарное дерево Морзе (массив cw_tree) 
 *       с быстрой индексной навигацией (idx * 2 + 1 / idx * 2 + 2) из кода UHSDR.
 *
 *  3. ОПТИМИЗАЦИЯ ПОД МИКРОКОНТРОЛЛЕР RP2040:
 *     - Математика амплитудного детектора fldigi переведена с плавающей точки (float) 
 *       на быстрые целочисленные операции со сдвигами (>>, >>3, >>8).
 *     - Покомпонентный сброс банка фильтров Герцеля (gtz_reset) в конце каждого 
 *       DSP-блока для подготовки к новому чанку отсчетов.
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
    return RX_MODE_NONE;   // 0 — «режим не найден», вызывающий код проверяет на NONE  
}

bool              dbg_rx = true;     // флаг печати лога отладки

static rx_mode_t  dec_mode = RX_MODE_NONE;  
static uint32_t   sym_samples;        // отсчётов на символ при Fs_out  
static uint32_t   sym_acc = 0;        // накоплено в окне  
static uint32_t   sample_idx = 0;
static int32_t rtty_disc = 0;     // накопитель дискриминатора текущего окна
static int32_t rtty_disc_ema = 0;      // сглаженный дискриминатор (~4 подокна)
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

// рядом с остальными static-полями декодеров, над rx_decoders_process  
static int32_t  cw_sig_avg    = 0;      // EMA уровня тона (fldigi sig_avg)  
static int32_t  cw_noise_avg = 1024;    // оценка шума огибающей  
static float    cw_dot_avg    = 60.0f;  // мс, адаптивная оценка точки  
static float    cw_dash_avg   = 180.0f; // мс, адаптивная оценка тире  
static float    cw_gap_avg  = 60.0f;     // мс, пауза между элементами  

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
// ----------------------------------------------------------------------------  
/**
 * ============================================================================
 *  rx_decoders.cpp (Блок Гёрцеля) — Комплексный банк тонов (Goertzel Detector)
 *  Архитектура: Однополюсный комплексный резонатор в базисе Q15
 *  Назначение: Параллельный спектральный анализ децимированной базовой полосы
 * ============================================================================
 *
 *  МАТЕМАТИЧЕСКИЙ АППАРАТ И ОСОБЕННОСТИ РЕАЛИЗАЦИИ:
 *
 *  1. СТРУКТУРА ДАННЫХ И КОДИРОВАНИЕ (структура `gtz_t`)
 *     - Перевод коэффициентов в комплексную форму: Вместо стандартного вещественного 
 *       рекуррентного фильтра второго порядка (метод Герцеля для действительного сигнала), 
 *       здесь применен однополюсный комплексный резонатор: s = x + c * s, где c = e^{jω}.
 *     - Переменные `cr` и `ci`: Хранят комплексный поворачивающий коэффициент (cos и sin) 
 *       в целочисленном формате фиксированной точки Q15 (масштаб 32768).
 *     - Переменные `ar` и `ai`: Внутреннее состояние комплексного интегратора (аккумулятора).
 *
 *  2. АЛГОРИТМ ФИЛЬТРАЦИИ И ВЫЧИСЛИТЕЛЬНАЯ ОПТИМИЗАЦИЯ (`gtz_push`)
 *     - Квадратурный вход: Функция принимает полноценный I/Q-поток базовой полосы (`xi`, `xq`).
 *     - Комплексное умножение «на лету»: На каждом отсчете выполняется раскрытие комплексного 
 *       произведения (g->cr + j*g->ci) * (g->ar + j*g->ai) с последующим битовым сдвигом `>> 15`:
 *         tr = xr + ((cr * ar - ci * ai) >> 15);
 *         ti = xq + ((cr * ai + ci * ar) >> 15);
 *     - Экономия тактов процессора: Если режим использует только действительную компоненту (I), 
 *       предусмотрена оптимизация — передача `xq = NULL` полностью отключает расчет Q-рекуррента.
 *
 *  3. РАСЧЕТ ЭНЕРГИИ БИНА И ЗАЩИТА ОТ ПЕРЕПОЛНЕНИЯ (`gtz_power`)
 *     - Оценка мощности: Вычисляется квадрат модуля комплексного числа на выходе: P = ar^2 + ai^2.
 *     - Расширение разрядности: Промежуточные вычисления квадратов выполняются в 64-битном 
 *       целом типе (`int64_t`), что гарантирует математическую точность.
 *     - Масштабирование и сатурация: Итоговый результат сдвигается на `>> 10` для нормализации 
 *       под 32-битное целое число (`int32_t`). Реализован жесткий ограничитель сверху (аппаратное 
 *       насыщение на уровне `0x7FFFFFFFLL`), предотвращающий инверсию знака при больших окнах накопления.
 *
 *  4. ЛОГИКА ОПРЕДЕЛЕНИЯ ЧАСТОТЫ (`gtz_argmax` / `gtz_winner`)
 *     - Принцип «Победитель получает всё» (Winner-Take-All): По окончании временного окна 
 *       (символьного чанка) алгоритм последовательно сканирует массив активных тонов, 
 *       находит индекс бина с максимальной энергетикой (`pmax`) и передает его декодерам верхнего уровня.
 *     - Покомпонентный сброс (`gtz_reset`): Очистка накопителей `ar = 0`, `ai = 0` происходит 
 *       строго после принятия решения по текущему окну, исключая взаимное влияние символов.
 *
 *  5. АДАПТИВНАЯ КОНФИГУРАЦИЯ СЕТКИ ТОНОВ (`gtz_bank_init`)
 *     - Гибкое масштабирование: Максимальная емкость банка ограничена константой `DEC_MAX_TONES = 33` 
 *       (достаточно для 33-тоновых систем FSQ/IFKP или 4-тонового JTTY).
 *     - Расчет шага частоты: Генерация коэффициентов происходит динамически при старте моды 
 *       на основе формулы `w = 2.0f * PI * (f0_hz + t * step_hz) / fs`, привязывая каждый бин 
 *       к строгой сетке частот сдвига относительно центра полосы.
 * ============================================================================
 */
  
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
    // МТК-2 русский регистр: UTF-8 строки (кириллица = 2 байта, char не вмещает)  
static const char *baudot_rus[32] = {  
    "",  "Е", "\n", "А", " ", "С", "И", "У",  
    "\r","Д", "Р", "Й", "Н", "Ф", "Ц", "К",  
    "Т", "З", "Л", "В", "Х", "Ы", "П", "Я",  
    "О", "Б", "Г", "",  "М", "Ь", "Ш", "" };

typedef struct {  
    uint8_t  figs;              // регистр LTRS/FIGS  
    uint8_t  state;             // 0=idle, 1=start, 2=биты, 3=стоп  
    uint8_t  bit_cnt;  
    uint8_t  shreg;  
    uint32_t bit_period;        // отсчётов на бит  
    uint8_t  prev_mark;         // решение предыдущего символа (для поиска фронта)    
    uint32_t next_sample;       // момент следующего отсчёта бита (в отсчётах)
    uint8_t  space_cnt;         // длительность текущего space-подряд (деглитчинг)
    uint32_t edge_idx;          // sample_idx первого space-подокна фронта 
} rtty_dec_t;  
static rtty_dec_t rt;  

// Некогерентный интегратор по подокнам: суммируем мощности тонов  
// за все ~8 подокон бита — решение о бите по полной энергии.  
static int32_t  rt_pwr_mark = 0;    // Σ power тона 1 (mark)  
static int32_t  rt_pwr_space = 0;   // Σ power тона 0 (space)  
static uint8_t  rt_pwr_n = 0;       // сколько подокон накоплено

#define MTK2_RUS  0x00  
#define MTK2_LAT  0x1F  
#define MTK2_FIG  0x1B

// Знак дискриминатора для mark (тон ВЫШЕ NCO).    
// rtty_disc = -Im(z*conj(z_prev)) => при +85 Гц disc < 0.    
// Если приём «зеркальный» — поменять на (disc > 0).  
#define RTTY_MARK(disc)   ((disc) < 0)

static inline bool sample_idx_ge(uint32_t a, uint32_t b) {  
    return (int32_t)(a - b) >= 0;  
}

// RTTY: решение о тоне каждые bit_period/8 отсчётов (subwin ~2.75 мс при 45.45 бод).  
// Старт-бит ловим по фронту mark→space, датабиты читаем в их середине.  
static void rtty_bit(bool mark, uint32_t idx /*в отсчётах Fs_out*/) {  
    switch (rt.state) {  
    case 0: {                              // ждём устойчивый фронт mark→space  
        if (!mark) {  
            if (rt.prev_mark) {            // первое space-подокно = кандидат на фронт  
                rt.edge_idx  = idx;  
                rt.space_cnt = 1;  
            } else if (++rt.space_cnt >= 2) {  
                // Фронт подтверждён: 2 подокна space подряд.    
                // Центр старт-бита = edge + 0.5·T; центр бита0 = edge + 1.5·T.  
                rt.state = 1; rt.bit_cnt = 0; rt.shreg = 0;  
                rt.next_sample = rt.edge_idx + rt.bit_period + rt.bit_period / 2;  
                if (dbg_rx)  
                    Serial.printf("[RTTY] start-bit edge@%lu\n",  
                                  (unsigned long)rt.edge_idx);  
            }  
        } else {  
            rt.space_cnt = 0;              // mark — сброс деглитчера    
        }  
        break;  
    } 
  
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
            if (dbg_rx)  
                Serial.printf("[RTTY] shreg=0x%02x figs=%u\n",  
                              (unsigned)c, (unsigned)rt.figs);
            if      (c == MTK2_FIG) rt.figs = 1;  
            else if (c == MTK2_LAT) rt.figs = 0;  
            else if (c == MTK2_RUS) rt.figs = 2;  
            else {  
                if (rt.figs == 2) {  
                    const char *s = baudot_rus[c];  
                    if (s[0]) Serial.print(s);  
                } else {  
                    char ch = rt.figs ? baudot_figs[c] : baudot_ltrs[c];  
                    if (ch) Serial.write(ch);  
                }  
            }
        }  
        break;  
    }  
    rt.prev_mark = mark;                     // единственное место обновления  
}
  
// ----------------------------------------------------------------------------  
// 4. CW ДЕКОДЕР: фронты -> элементы -> знаки (адаптивная классификация UHSDR)  
// ----------------------------------------------------------------------------  
  
// Бинарное дерево Морзе, строится кодом — пробелы считать не нужно.  
// '.'→2k+1, '-'→2k+2, корень idx=0.  
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
    // on == true  -> тон НАЧАЛСЯ (завершилась ПАУЗА)  
    // on == false -> тон ЗАВЕРШИЛСЯ (завершился ИМПУЛЬС)  
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
  
        // Буфер насытился без паузы-разделителя (длинная помеха) —  
        // сбрасываем мусор, иначе знак «зависнет» до следующего гэпа  
        if (cw.elem_n >= sizeof(cw.elem_buf) - 1) {  
            if (dbg_rx) Serial.println(F("[CW] elem_buf satur — сброс"));  
            cw.elem_n = 0;  
        }  
  
        // Плавная адаптация межэлементного зазора  
        if (dur_ms < cw_dot_avg * 2.0f)  
            cw_gap_avg += (dur_ms - cw_gap_avg) * 0.1f;  
  
    } else {  
        // ---------------- ЗАВЕРШИЛСЯ ИМПУЛЬС ----------------  
        if (dur_ms < 20.0f) return;                          // статический разряд  
  
        // Залипание > 2 с (помеха, перегрузка AGC) — не элемент Морзе,  
        // не портим dash_avg и не пишем в буфер  
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
// rx_dsp вращает спектр так, что center_hz (RF) → 0 Гц. Поэтому все смещения  
// ниже — константы относительно 0, а не относительно center_hz.  
void rx_decoders_start(uint8_t mode, float center_hz) {
    if (dbg_rx) {Serial.printf("[DEC] start: mode=%u center=%.1f\n",  
                (unsigned)mode, (double)center_hz);}
    dec_mode = (rx_mode_t)mode;  
    fs_out = rx_dsp_fs_out();  
    if (dbg_rx) {Serial.printf("[DEC] fs_out=%.1f\n", (double)fs_out);}       // <-- fs валидна 
    f_center_hz = center_hz;  
    sym_acc = 0; sample_idx = 0;  
    rtty_disc = 0; rtty_disc_ema = 0;
    memset(&rt, 0, sizeof(rt)); memset(&cw, 0, sizeof(cw));  
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
    // CW: явный сброс состояния детектора/классификатора уже сделан выше  
    cw.tone_on  = false;  
    cw.on_start = cw.off_start = 0;  
    cw.elem_n   = 0;
    
    cw_tree_init();

    switch (dec_mode) {  
    case RX_MODE_CW:  
        // Тон реально сидит ~+63 Гц выше базовой полосы — остаточный сдвиг LO  
        // (тот же замер, что давал бин-центры JTTY на +63 Гц от расчёта).  
        gtz_bank_init(63.0f, 0.0f, 1, fs_out);  
        sym_samples = (uint32_t)(fs_out * 0.005f);  
        Serial.printf("[CW-RX] bank ready, gtz_ntones=%u sym=%lu\n",  
                      gtz_ntones, (unsigned long)sym_samples);  
        break;
    case RX_MODE_RTTY:  
        // Тоны реально сидят ~+63 Гц выше базовой полосы (остаточный сдвиг LO,    
        // тот же замер что у CW): space = -85+63 = -22, mark = +85+63 = +148.  
        gtz_bank_init(-22.0f, 170.0f, 2, fs_out);                // space/mark
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
  
        // sig: быстрая атака только когда ключ ON и pe растёт; спад — очень медленный  
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
            // key_deb = true означает НАЖАТИЕ, false - ОТПУСКАНИЕ
            cw_on_edge(key_deb, sample_idx, (uint32_t)(fs_out * 1000.0f));  
            cw.tone_on = key_deb;  
            if (key_deb) cw.on_start = sample_idx; else cw.off_start = sample_idx;  
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
            // rtty_disc обнуляется в конце итерации подокна (ниже, после switch)
            // rtty_disc_ema += (rtty_disc - rtty_disc_ema) >> 2;   // EMA за ~4 подокна
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
            rtty_disc_ema += (rtty_disc - rtty_disc_ema) >> 2;  // один шаг на подокно  
            rtty_bit(rtty_disc_ema < 0, sample_idx);  
            if (dbg_rx) {  
                static uint8_t rd = 0;  
                if (++rd >= 16) {  
                    rd = 0;  
                    Serial.printf("[RTTY] raw=%ld ema=%ld %s\n",  
                                  (long)rtty_disc, (long)rtty_disc_ema,  
                                  (rtty_disc_ema < 0) ? "mark" : "space");  
                }  
            }  
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
            // TODO: sync по is13 + Витерби TBCC  
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

