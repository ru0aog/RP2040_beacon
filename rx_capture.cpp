/**  
 * ============================================================================  
 *  rx_capture.cpp — DMA-движок приёмной части (Двухканальный бесшовный Ping-Pong DMA)
 *  Версия 2.24 от 2026-10-14, автор RU0AOG  
 * ============================================================================ 
 *  
 *  Два DMA-канала (A→B→A по chain_to), блоки по 512 32-битных слов [I|Q].  
 *  Скорость = RX_ADC_SAMPLE_RATE_HZ слов/с (480k по умолчанию = 240 ksps/канал).
 * 
 * ============================================================================  
 */
  
#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "hardware/adc.h"  
#include "hardware/dma.h"  
#include "hardware/irq.h"  

// Два буфера ping-pong: каждый — RX_BLOCK_SAMPLES слов по 32 бита  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_a[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_b[RX_BLOCK_SAMPLES];  
  
// Lock-free очередь готовых блоков (глубина 4: два DMA + запас)  
#define RX_QUEUE_DEPTH 4  
static volatile uint32_t* rx_queue[RX_QUEUE_DEPTH];  
static volatile uint8_t   rx_q_head = 0;   // пишет IRQ  
static volatile uint8_t   rx_q_tail = 0;   // читает consumer  
static volatile uint32_t  rx_dropped = 0;  // счётчик переполнений (диагностика)  

// Флаги «буфер ещё у потребителя» — детект перезаписи  
static volatile bool rx_buf_a_busy = false;  
static volatile bool rx_buf_b_busy = false; 
  
static int rx_dma_a = -1, rx_dma_b = -1;  
static volatile bool rx_running = false;  
  
// ---------------------------------------------------------------  
// Обработчик DMA IRQ0 (работает на том ядре, где включён — core0).  
// Когда канал закончил блок, он сам чейнится на другой канал;  
// мы: квитируем IRQ, блокируем повторный запуск ЭТОГО канала  
// до возврата буфера потребителем, пушим буфер в очередь.  
// ---------------------------------------------------------------  
static void __not_in_flash_func(rx_dma_irq)(void) {  
    // Канал A завершил блок: hardware сама запустила B через chain_to,  
    // канал A остановлен. Аборт не нужен — он и создавал гонку.  
    if (dma_channel_get_irq0_status(rx_dma_a)) {  
        dma_channel_acknowledge_irq0(rx_dma_a);
        // WRITE_ADDR не перезагружается по chain_to — вернуть до следующего  
        // триггера A (он придёт только после завершения B, время есть)  
        dma_channel_set_write_addr(rx_dma_a, (void*)rx_buf_a, false);
        if (rx_buf_a_busy) {  
            rx_dropped++;               // consumer не успел — buf_a перезаписан  
        } else {  
            rx_buf_a_busy = true;  
            uint8_t next = (rx_q_head + 1) % RX_QUEUE_DEPTH;  
            if (next != rx_q_tail) {  
                rx_queue[rx_q_head] = (uint32_t*)rx_buf_a;  
                rx_q_head = next;  
            } else { rx_dropped++; rx_buf_a_busy = false; }  
        }  
    }  
    if (dma_channel_get_irq0_status(rx_dma_b)) {  
        dma_channel_acknowledge_irq0(rx_dma_b);
        dma_channel_set_write_addr(rx_dma_b, (void*)rx_buf_b, false);
        if (rx_buf_b_busy) {  
            rx_dropped++;  
        } else {  
            rx_buf_b_busy = true;  
            uint8_t next = (rx_q_head + 1) % RX_QUEUE_DEPTH;  
            if (next != rx_q_tail) {  
                rx_queue[rx_q_head] = (uint32_t*)rx_buf_b;  
                rx_q_head = next;  
            } else { rx_dropped++; rx_buf_b_busy = false; }  
        }  
    }  
}
  
void rx_capture_start(void) {  
    rx_q_head = rx_q_tail = 0;  // сброс флагов
    rx_dropped = 0;
    rx_buf_a_busy = rx_buf_b_busy = false;
  
    adc_init();  
    adc_gpio_init(RX_ADC_PIN_I);  
    adc_gpio_init(RX_ADC_PIN_Q);  
    adc_set_round_robin(0b00011);                    // ADC0, ADC1 по кругу  
    // FIFO: enable, DREQ, порог 8 слов (4 пары I/Q), без err-бита, без сдвига  
    adc_fifo_setup(true, true, 8, false, false);  
    adc_set_clkdiv(48000000.0f / RX_ADC_SAMPLE_RATE_HZ);  
  
    rx_dma_a = dma_claim_unused_channel(true);  
    rx_dma_b = dma_claim_unused_channel(true);  
  
    dma_channel_config ca = dma_channel_get_default_config(rx_dma_a);  
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_32);  
    channel_config_set_read_increment(&ca, false);    // читаем adc_fifo  
    channel_config_set_write_increment(&ca, true);  
    channel_config_set_dreq(&ca, DREQ_ADC);  
    channel_config_set_chain_to(&ca, rx_dma_b);       // по завершении → канал B  
  
    dma_channel_config cb = dma_channel_get_default_config(rx_dma_b);  
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_32);  
    channel_config_set_read_increment(&cb, false);  
    channel_config_set_write_increment(&cb, true);  
    channel_config_set_dreq(&cb, DREQ_ADC);  
    channel_config_set_chain_to(&cb, rx_dma_a);       // по завершении → канал A  
  
    dma_channel_configure(rx_dma_a, &ca, (void*)rx_buf_a, &adc_hw->fifo,  
                          RX_BLOCK_SAMPLES, false);  
    dma_channel_configure(rx_dma_b, &cb, (void*)rx_buf_b, &adc_hw->fifo,  
                          RX_BLOCK_SAMPLES, false);  
  
    dma_channel_acknowledge_irq0(rx_dma_a);  
    dma_channel_acknowledge_irq0(rx_dma_b);  
    dma_channel_set_irq0_enabled(rx_dma_a, true);  
    dma_channel_set_irq0_enabled(rx_dma_b, true);  
    irq_set_exclusive_handler(DMA_IRQ_0, rx_dma_irq);  
    irq_set_enabled(DMA_IRQ_0, true);  
  
    adc_fifo_drain();  
    adc_select_input(0);                              // round-robin начнёт с ADC0  
    adc_run(true);  
    rx_running = true;  
    dma_channel_start(rx_dma_a);                      // стартует A, дальше чейн  

    // rx_capture.cpp, в конце rx_capture_start()  
    io_rw_32 ctrl = dma_channel_hw_addr(rx_dma_a)->ctrl_trig;  
    Serial.printf("[CAP] chA=%d chB=%d  ctrl=0x%08lX  size=%lu incr_r=%d incr_w=%d dreq=%lu tc=%lu\n",  
                    (int)rx_dma_a, (int)rx_dma_b,  
                    (unsigned long)ctrl,  
                    (unsigned long)((ctrl >> 2) & 3),    // DATA_SIZE: 0=8, 1=16, 2=32 бита  
                    (int)((ctrl >> 4) & 1),              // INCR_READ  (0 — читаем FIFO)  
                    (int)((ctrl >> 5) & 1),              // INCR_WRITE (1 — пишем в буфер)  
                    (unsigned long)((ctrl >> 15) & 0x3F),// TREQ_SEL (DREQ_ADC = 36)  
                    (unsigned long)dma_channel_hw_addr(rx_dma_a)->transfer_count);  
    // контрольный дамп al1_ctrl — то, что реально уедет в канал по trigger  
    Serial.printf("[CAP] ctrl=0x%08lX\n", (unsigned long)dma_hw->ch[rx_dma_a].al1_ctrl);

}  
  
void rx_capture_stop(void) {  
    rx_running = false;  
    adc_run(false);  
    dma_channel_abort(rx_dma_a);  
    dma_channel_abort(rx_dma_b);  
    dma_channel_set_irq0_enabled(rx_dma_a, false);  
    dma_channel_set_irq0_enabled(rx_dma_b, false);  
    adc_fifo_drain();  
    rx_q_head = rx_q_tail = 0;  
}  
  
// --- Интерфейс для DSP (вызывается на core1) ---  
// Возвращает указатель на блок 32-битных слов [I|Q] или NULL.  
const volatile uint32_t* rx_capture_get_block(void) {  
    if (rx_q_tail == rx_q_head) return NULL;  
    return rx_queue[rx_q_tail];  
}  
  
// Освобождает блок. DMA здесь НЕ трогаем: канал будет перезапущен  
// аппаратно по chain_to при завершении напарника — ручной старт  
// вызывал двойной запуск и запись в занятый буфер.  
void rx_capture_release(void) {  
    const volatile uint32_t* buf = rx_queue[rx_q_tail];  
    rx_q_tail = (rx_q_tail + 1) % RX_QUEUE_DEPTH;  
    if (buf == rx_buf_a)      rx_buf_a_busy = false;  
    else if (buf == rx_buf_b) rx_buf_b_busy = false;  
}
  
uint32_t rx_capture_dropped(void) { return rx_dropped; }




