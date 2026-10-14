#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "pico/multicore.h"  
  
// ---- Параметры тракта ----  
#define RX_BLOCK_SAMPLES      512                   // должен совпадать с rx_capture.cpp  
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
} hb_state_t;  
  
static hb_state_t hb_i[RX_DECIM_STAGES], hb_q[RX_DECIM_STAGES];  
  
// Один отсчёт через HB-фильтр; возвращает true, когда на выходе  
// готов прореженный отсчёт (каждый второй вход).  
static inline bool hb_push(hb_state_t* st, int32_t x, int32_t* out, uint8_t* phase) {  
    st->d[st->w] = x;  
    st->w = (st->w + 1) % HB_NTAPS;  
    (*phase) ^= 1;  
    if (*phase) return false;               // нечётный вход — выхода нет  
    int32_t acc = 0;  
    for (int k = 0; k < HB_NTAPS; k++) {  
        int idx = (int)st->w + k; if (idx >= HB_NTAPS) idx -= HB_NTAPS;  
        acc += (int32_t)st->d[idx] * hb_taps[k];  
    }  
    *out = acc >> 15;                       // Q15 → int  
    return true;  
}  
  
// ---- Выходные буферы базовой полосы ----  
#define RX_OUT_SAMPLES   (RX_BLOCK_SAMPLES / 8)   // 64 пары I/Q за блок  
static int16_t out_i[RX_OUT_SAMPLES], out_q[RX_OUT_SAMPLES];  
static volatile uint8_t out_ready = 0;  
  
// Точка входа DSP на core1  
static void __not_in_flash_func(rx_dsp_entry)(void) {  
    uint8_t ph[RX_DECIM_STAGES] = {0};  
    while (true) {  
        const volatile uint32_t* blk = rx_capture_get_block();  
        if (!blk) { tight_loop_contents(); continue; }  
  
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
                bool ri = hb_push(&hb_i[s], vi, &oi, &ph[s]);  
                hb_push(&hb_q[s], vq, &oq, &ph[s]);            // тот же фазовый бит  
                if (!ri) { emit = false; break; }  
                vi = oi; vq = oq;  
            }  
            if (emit && n_out < RX_OUT_SAMPLES) {  
                // Насыщение к int16 (аналог __SSAT(16)), без зависимости от CMSIS  
                out_i[n_out] = (int16_t)(vi > 32767 ? 32767 : (vi < -32768 ? -32768 : vi));  
                out_q[n_out] = (int16_t)(vq > 32767 ? 32767 : (vq < -32768 ? -32768 : vq));
                n_out++;  
            }  
        }  
        rx_capture_release();  
        out_ready = 1;          // consumer (декодер/FFT) забирает out_i/out_q  
    }  
}  
  
// ---- Публичный API ----  
void rx_dsp_start(float if_hz) {  
    // f_nco = ПЧ + поправка квантования LO:  
    // сигнал сидит на +RX_IF_SHIFT_HZ минус ошибка LO  
    float f_nco = if_hz;   // точнее: RX_IF_SHIFT_HZ - (lo_nominal - lo_actual)  
    nco_step = (uint32_t)(4294967296.0f * f_nco / RX_FS_IN_HZ);  
    nco_phase = 0;  
    memset(hb_i, 0, sizeof(hb_i));  
    memset(hb_q, 0, sizeof(hb_q));  
    nco_lut_init();  
    multicore_reset_core1();  
    multicore_launch_core1(rx_dsp_entry);  
}  
  
void rx_dsp_set_nco_hz(float f_hz) {  
    nco_step = (uint32_t)(4294967296.0f * f_hz / RX_FS_IN_HZ);  
}  
  
bool rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n) {  
    if (!out_ready) return false;  
    *i = out_i; *q = out_q; *n = RX_OUT_SAMPLES;  
    return true;  
}  
void rx_dsp_release_output(void) { out_ready = 0; }