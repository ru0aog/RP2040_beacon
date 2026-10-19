/**  
 * ============================================================================  
 *  rx_dsp.cpp — модуль децимирующего DSP-тракта (core1)  
 *  Версия 2.25 от 2026-10-14, автор RU0AOG  
 *  
 *  Тракт: 32-бит слова [I|Q] из rx_capture → центрирование → NCO-сдвиг →  
 *  3 каскада half-band FIR (/8) → SPSC-очередь базовой полосы  
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
  
// ---- Выходная SPSC-очередь базовой полосы: core1 -> core0 ----  
// head изменяет только producer (core1), tail — только consumer (core0).  
// Кольцо из 4 слотов имеет вместимость 3 блока: один слот резервируется,  
// чтобы отличать полное состояние от пустого.  
// Исправление двойной буферизации: раньше consumer всегда читал  
// ПОСЛЕДНИЙ опубликованный буфер → дубли/пропуски блоков → блочная  
// модуляция на частоте блоков Fs/64 ≈ 244 Гц (гребёнка в спектре).  
#define RX_OUT_SAMPLES   (RX_BLOCK_SAMPLES / 8)   // 64 пары I/Q за блок  
#define RX_OUT_BUFS      4  
  
static int16_t out_i[RX_OUT_BUFS][RX_OUT_SAMPLES];  
static int16_t out_q[RX_OUT_BUFS][RX_OUT_SAMPLES];  
static volatile uint8_t  out_head = 0;    // запись: только core1  
static volatile uint8_t  out_tail = 0;    // запись: только core0  
static volatile uint32_t out_dropped = 0; // диагностика переполнений  
  
// Точка входа DSP на core1  
static void __not_in_flash_func(rx_dsp_entry)(void) {  
    // Оценка DC-смещения АЦП: EMA по входным отсчётам.  
    // alpha=1/256 → постоянная ~0.4 с, сигнал тона за символ усредняется в 0.  
    static int32_t dc_i = 2048 << 8, dc_q = 2048 << 8;  
  
    while (true) {  
        watchdog_update();  
        const volatile uint32_t* blk = rx_capture_get_block();  
        if (!blk) { tight_loop_contents(); watchdog_update(); continue; }  
  
        uint8_t wr        = out_head;  
        uint8_t next_head = (uint8_t)((wr + 1) % RX_OUT_BUFS);  
        // Слот head — резервный: при полной очереди входной блок всё равно  
        // обрабатываем (NCO/DC/FIR не разрываются), но результат не публикуем.  
        bool    queue_full = (next_head == out_tail);  
        uint8_t n_out = 0;  
  
        // DMA_SIZE_16: каждое 32-битное слово FIFO = пара отсчётов  
        // [15:0] = ADC0 (I), [31:16] = ADC1 (Q). Пар в блоке = RX_BLOCK_SAMPLES.  
        const volatile int16_t* s = (const volatile int16_t*)blk;  
  
        for (int n = 0; n < RX_BLOCK_SAMPLES; n++) {  
            int32_t i_meas = s[2 * n];  
            int32_t q_meas = s[2 * n + 1];  
  
            // EMA DC и центрирование  
            dc_i += (i_meas - (dc_i >> 8));        // dc в Q8  
            dc_q += (q_meas - (dc_q >> 8));  
            int32_t i_raw = i_meas - (dc_i >> 8);  
            int32_t q_raw = q_meas - (dc_q >> 8);  
  
            // Комплексное вращение NCO: сдвиг спектра на -f_nco  
            int16_t sn, cs;  
            nco_sincos(nco_phase, &sn, &cs);  
            nco_phase += nco_step;  
            int32_t i_rot = (i_raw * cs + q_raw * sn) >> 15;  
            int32_t q_rot = (q_raw * cs - i_raw * sn) >> 15;  
  
            // Каскад HB-децимации /8  
            int32_t vi = i_rot, vq = q_rot;  
            bool emit = true;  
            for (int st = 0; st < RX_DECIM_STAGES; st++) {  
                int32_t oi, oq;  
                bool ri = hb_push(&hb_i[st], vi, &oi);  
                bool rq = hb_push(&hb_q[st], vq, &oq);  
                if (!ri || !rq) { emit = false; break; }  
                vi = oi; vq = oq;  
            }  
            if (emit && !queue_full && n_out < RX_OUT_SAMPLES) {  
                out_i[wr][n_out] = sat16(vi);  
                out_q[wr][n_out] = sat16(vq);  
                n_out++;  
            }  
        }  
        rx_capture_release();  
  
        if (queue_full) {  
            out_dropped++;            // блок выхода потерян, вход обработан  
        } else {  
            __dmb();                  // данные блока видимы до публикации head  
            out_head = next_head;  
        }  
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
  
    Serial.printf("[RXDBG] target=%.1f  lo_actual=%lu  f_nco=%.3f  nco_step=0x%08lX\n",  
                  (double)f_target_hz,  
                  (unsigned long)vfo_quadrature_lo_hz(),  
                  (double)f_nco,  
                  (unsigned long)nco_step);  
  
    // Сначала гасим core1 — иначе он может перезаписать буферы/состояния  
    // между memset и reset, и очередь стартует в грязном состоянии.  
    multicore_reset_core1();  
  
    memset(hb_i, 0, sizeof(hb_i));  
    memset(hb_q, 0, sizeof(hb_q));  
    memset(out_i, 0, sizeof(out_i));  
    memset(out_q, 0, sizeof(out_q));  
    out_head = out_tail = 0;  
    out_dropped = 0;  
  
    nco_lut_init();  
    multicore_launch_core1(rx_dsp_entry);  
}  
  
// AFC/тонкая подстройка без перезапуска тракта  
void rx_dsp_set_nco_hz(float f_hz) {  
    nco_step = (uint32_t)(4294967296.0f * f_hz / RX_FS_IN_HZ);  
}  
  
// Отдаёт ОЧЕРЕДНОЙ заполненный буфер (FIFO по tail); consumer обязан  
// вызвать rx_dsp_release_output() после чтения — ровно один раз на блок.  
bool rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n) {  
    uint8_t tail = out_tail;  
    if (tail == out_head) return false;    // очередь пуста  
  
    // После наблюдения нового head данные опубликованного блока  
    // должны быть видимы до того, как consumer начнёт их читать.  
    __dmb();  
    *i = out_i[tail];  
    *q = out_q[tail];  
    *n = RX_OUT_SAMPLES;  
    return true;  
}  
  
void rx_dsp_release_output(void) {  
    // Все чтения текущего блока должны завершиться до освобождения слота.  
    __dmb();  
    out_tail = (uint8_t)((out_tail + 1) % RX_OUT_BUFS);  
}  
  
// Счётчик потерянных выходных блоков — для диагностики захвата.  
// Для чистого спектрального измерения приращение за захват должно быть 0.  
uint32_t rx_dsp_out_dropped(void) { return out_dropped; }  
  
// Возвращает Fs выхода базовой полосы — нужна декодерам для таймингов  
float rx_dsp_fs_out(void) { return RX_FS_OUT_HZ; }  
  
// Останавливает DSP-тракт: снимает core1 и гасит выходную очередь.  
// Вызывается из rx off до rx_capture_stop — иначе core1 продолжит  
// крутиться на освобождённых DMA-буферах.  
void rx_dsp_stop(void) {  
    multicore_reset_core1();   // гасим DSP-ядро  
    out_head = out_tail = 0;  
    out_dropped = 0;  
}