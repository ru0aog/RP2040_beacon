/**    
 * ============================================================================    
 *  rx_dsp.cpp — модуль децимирующего DSP-тракта (core1)    
 *  Версия 2.25 от 2026-10-14, автор RU0AOG    
 *    
 *  Тракт: 32-бит слова [I|Q] из rx_capture → центрирование → NCO-сдвиг →    
 *  3 каскада half-band FIR (/8) → дублированные буферы базовой полосы    
 *  int16 I/Q по 64 отсчёта на блок.    
 *    
 *  Fs на канал = RX_ADC_SAMPLE_RATE_HZ/2 (~125 ksps при 250k слов/с),    
 *  Fs_out = Fs/8 ≈ 15.6 ksps на канал (Nyquist ±7.8 кГц — покрывает    
 *  полосу всех узкополосных модов и ПЧ-сдвиг).    
 * ============================================================================    
 */  

#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "pico/multicore.h"  
  
// ---- Параметры тракта ----  
#define RX_FS_IN_HZ           (RX_ADC_SAMPLE_RATE_HZ / 2.0f)  // Fs на канал
#define RX_DECIM_STAGES       3                     // 3 half-band каскада: /8  
#define RX_FS_OUT_HZ          (RX_FS_IN_HZ / 8.0f)  // ~15.6 ksps при 250k  
  
// NCO: 32-битный фазовый аккумулятор  
static volatile uint32_t nco_phase = 0;  
static volatile uint32_t nco_step  = 0;     // 2^32 * f_nco / Fs_in  
  
// sin/cos LUT: 1024 точки, Q15, квадратурная пара  
static int16_t nco_lut[1024];  
static void nco_lut_init(void) {  
    for (int i = 0; i < 1024; i++)  
        nco_lut[i] = (int16_t)(32767.0f * sinf(2.0f * PI * i / 1024.0f));  
}  
static inline void nco_sincos(uint32_t ph, int16_t* s, int16_t* c) {  
    uint16_t idx = ph >> 22;                // старшие 10 бит фазы  
    *s = nco_lut[idx];  
    *c = nco_lut[(idx + 256) & 1023];       // cos = sin сдвинутый на 90°  
}  
  
// ---- Half-band FIR, 15 отводов, коэффициенты для децимации на 2 ----  
// Стандартный HB-фильтр: чётные отводы (кроме центра) = 0.  
// Q15-коэффициенты (окно на классическом HB):  
static const int16_t hb_taps[15] = {  
    -39, 0, 294, 0, -1076, 0, 10375, 16384,  
    10375, 0, -1076, 0, 294, 0, -39  
};  
#define HB_NTAPS 15  
  
typedef struct {  
    int32_t d[HB_NTAPS];   // линия задержки (накопленные I или Q)  
    uint8_t w;             // указатель записи
    uint8_t phase;         // свой бит чётности на каждый канал — I и Q независимы
} hb_state_t;  
  
static hb_state_t hb_i[RX_DECIM_STAGES], hb_q[RX_DECIM_STAGES];  
  
// Один отсчёт через HB-фильтр; true, когда готов прореженный выход.  
static inline bool hb_push(hb_state_t* st, int32_t x, int32_t* out) {  
    st->d[st->w] = x;  
    st->w = (st->w + 1) % HB_NTAPS;  
    st->phase ^= 1;  
    if (st->phase) return false;      // нечётный вход — выхода нет  
    int32_t acc = 0;  
    for (int k = 0; k < HB_NTAPS; k++) {  
        int idx = (int)st->w + k; if (idx >= HB_NTAPS) idx -= HB_NTAPS;  
        acc += (int32_t)st->d[idx] * hb_taps[k];  
    }  
    *out = acc >> 15;  
    return true;  
}  

static inline int16_t sat16(int32_t v) {  
    return (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));  
}  

// ---- Выходные буферы базовой полосы: ДВОЙНАЯ буферизация ----  
// Пока декодер (core0) читает буфер X, DSP (core1) пишет буфер Y.  
// Без этого декодер обязан был успеть за один период блока (~2 мс).  
#define RX_OUT_SAMPLES   (RX_BLOCK_SAMPLES / 8)   // 64 пары I/Q за блок  
#define RX_OUT_BUFS      2  
  
static int16_t out_i[RX_OUT_BUFS][RX_OUT_SAMPLES];  
static int16_t out_q[RX_OUT_BUFS][RX_OUT_SAMPLES];  
static volatile uint8_t out_wr = 0;      // индекс буфера, куда пишет DSP  
static volatile uint8_t out_ready = 0;   // маска готовых буферов (биты 0..1)  
static volatile uint8_t out_read = 0;    // индекс буфера, отданного consumer'у  
  
// Точка входа DSP на core1  
static void __not_in_flash_func(rx_dsp_entry)(void) {  
    while (true) {  
        const volatile uint32_t* blk = rx_capture_get_block();  
        if (!blk) { tight_loop_contents(); continue; }  
  
        // Если consumer ещё не отдал предыдущий буфер — пишем в свободный,  
        // оба заняты не бывает: период блока 2 мс, обработка ~0.3 мс.  
        uint8_t wr = out_wr;  
  
        uint8_t n_out = 0;  
        for (int n = 0; n < RX_BLOCK_SAMPLES; n++) {  
            uint32_t w = blk[n];  
            int32_t i_raw = (int16_t)(w & 0xFFFF) - 2048;      // ADC0 → центрирование  
            int32_t q_raw = (int16_t)(w >> 16)     - 2048;     // ADC1  
  
            // Комплексное вращение NCO: сдвиг спектра на -f_nco  
            int16_t sn, cs;  
            nco_sincos(nco_phase, &sn, &cs);  
            nco_phase += nco_step;  
            // (I+jQ)·(cos - j·sin): I' = I·cos + Q·sin; Q' = Q·cos - I·sin  
            int32_t i_rot = (i_raw * cs + q_raw * sn) >> 15;  
            int32_t q_rot = (q_raw * cs - i_raw * sn) >> 15;  
  
            // Каскад HB-децимации /8  
            int32_t vi = i_rot, vq = q_rot;  
            bool emit = true;  
            for (int s = 0; s < RX_DECIM_STAGES; s++) {  
                int32_t oi, oq;  
                bool ri = hb_push(&hb_i[s], vi, &oi);  
                bool rq = hb_push(&hb_q[s], vq, &oq);  
                if (!ri || !rq) { emit = false; break; }  // фазы I/Q синхронны  
                vi = oi; vq = oq;  
            }  
            if (emit && n_out < RX_OUT_SAMPLES) {  
                out_i[wr][n_out] = sat16(vi);  
                out_q[wr][n_out] = sat16(vq);  
                n_out++;  
            }  
        }  
        rx_capture_release();  
  
        // Публикация: помечаем буфер готовым, переключаем указатель записи.  
        // Чтение/запись out_ready/out_wr — однобайтовые, атомарны между ядрами.  
        out_read  = wr;  
        out_ready |= (1u << wr);  
        out_wr    = wr ^ 1;  
    }  
}   
  
// ---- Публичный API ----  
  
// if_hz — ЖЕЛАЕМАЯ частота приёма (несущая мода). Точная ПЧ вычисляется
// от фактического квантованного LO: сигнал в базовой полосе сидит на
// f_if_actual = f_target - f_lo_actual.
void rx_dsp_start(float f_target_hz) {
    // Ошибка квантования LO вносится в NCO:
    float f_nco = f_target_hz - (float)vfo_quadrature_lo_hz();
    nco_step  = (uint32_t)(4294967296.0f * f_nco / RX_FS_IN_HZ);
    nco_phase = 0;
  
    memset(hb_i, 0, sizeof(hb_i));
    memset(hb_q, 0, sizeof(hb_q));
    memset(out_i, 0, sizeof(out_i));
    memset(out_q, 0, sizeof(out_q));
    out_wr = out_read = out_ready = 0;

    nco_lut_init();
    multicore_reset_core1();
    multicore_launch_core1(rx_dsp_entry);
}  
  
// AFC/тонкая подстройка без перезапуска тракта  
void rx_dsp_set_nco_hz(float f_hz) {  
    nco_step = (uint32_t)(4294967296.0f * f_hz / RX_FS_IN_HZ);  
}  
  
// Отдаёт СВЕЖИЙ заполненный буфер; consumer обязан вызвать  
// rx_dsp_release_output() после чтения.  
bool rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n) {  
    if (!out_ready) return false;  
    uint8_t rd = out_read;                    // последний опубликованный  
    *i = out_i[rd]; *q = out_q[rd]; *n = RX_OUT_SAMPLES;  
    return true;  
}  
void rx_dsp_release_output(void) {  
    out_ready &= ~(1u << out_read);  
}  
  
// Возвращает Fs выхода базовой полосы — нужна декодерам для таймингов  
float rx_dsp_fs_out(void) { return RX_FS_OUT_HZ; }




