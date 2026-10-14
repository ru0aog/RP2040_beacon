/**  
 * ============================================================================  
 *  rx_capture.cpp — Приёмная часть (Двухканальный бесшовный Ping-Pong DMA)
 *  Версия 2.24 от 2026-10-14, автор RU0AOG  
 * ============================================================================ 
 *  
 *  single DMA channel, 256×2 ping-pong, 250 ksps total - ?
 * 
 *  Для гарантированного исключения джиттера и фазовых сдвигов I/Q на RP2040 
 *  в режиме пинг-понга используется классическая пара из двух каналов DMA, 
 *  которые поочередно чейнятся друг на друга. 
 *  Пока один канал аппаратно и молча пишет в rx_buf[0], 
 *  второй канал ждет своей очереди, 
 *  а процессор в прерывании спокойно забирает данные без гонки за адреса.
 *  
 * ============================================================================  
 */

// Пины АЦП: I-канал GP26 (ADC0), Q-канал GP27 (ADC1)  
#define RX_ADC_PIN_I          26  
#define RX_ADC_PIN_Q          27  
  
#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "hardware/adc.h"  
#include "hardware/dma.h"  
#include "hardware/irq.h"  
  
// ---- Параметры захвата ----  
// Пины заданы в vfo_hardware.cpp: RX_ADC_PIN_I=26 (ADC0), RX_ADC_PIN_Q=27 (ADC1)  
// FIFO 32-битное слово: [31:16] — канал Q (ADC1), [15:0] — I (ADC0),  
// т.к. round-robin выдаёт отсчёты по возрастанию канала и FIFO пакует по 2.  
#define RX_BLOCK_SAMPLES   512          // отсчётов I/Q на блок (пар в 32-бит словах)  
  
// Два буфера ping-pong: каждый — RX_BLOCK_SAMPLES слов по 32 бита  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_a[RX_BLOCK_SAMPLES];  
static volatile uint32_t __attribute__((aligned(4))) rx_buf_b[RX_BLOCK_SAMPLES];  
  
// Lock-free очередь готовых блоков (глубина 4: два DMA + запас)  
#define RX_QUEUE_DEPTH 4  
static volatile uint32_t* rx_queue[RX_QUEUE_DEPTH];  
static volatile uint8_t   rx_q_head = 0;   // пишет IRQ  
static volatile uint8_t   rx_q_tail = 0;   // читает consumer  
static volatile uint32_t  rx_dropped = 0;  // счётчик переполнений (диагностика)  
  
static int rx_dma_a = -1, rx_dma_b = -1;  
static volatile bool rx_running = false;  
  
// ---------------------------------------------------------------  
// Обработчик DMA IRQ0 (работает на том ядре, где включён — core0).  
// Когда канал закончил блок, он сам чейнится на другой канал;  
// мы: квитируем IRQ, блокируем повторный запуск ЭТОГО канала  
// до возврата буфера потребителем, пушим буфер в очередь.  
// ---------------------------------------------------------------  
static void __not_in_flash_func(rx_dma_irq)(void) {  
    // Канал A завершил буфер A  
    if (dma_channel_get_irq0_status(rx_dma_a)) {  
        dma_channel_acknowledge_irq0(rx_dma_a);  
        dma_channel_abort(rx_dma_a);              // затормозить до освобождения буфера  
        uint8_t next = (rx_q_head + 1) % RX_QUEUE_DEPTH;  
        if (next != rx_q_tail) {  
            rx_queue[rx_q_head] = (uint32_t*)rx_buf_a;  
            rx_q_head = next;  
        } else {  
            rx_dropped++;  
        }  
    }  
    // Канал B завершил буфер B  
    if (dma_channel_get_irq0_status(rx_dma_b)) {  
        dma_channel_acknowledge_irq0(rx_dma_b);  
        dma_channel_abort(rx_dma_b);  
        uint8_t next = (rx_q_head + 1) % RX_QUEUE_DEPTH;  
        if (next != rx_q_tail) {  
            rx_queue[rx_q_head] = (uint32_t*)rx_buf_b;  
            rx_q_head = next;  
        } else {  
            rx_dropped++;  
        }  
    }  
}  
  
void rx_capture_start(void) {  
    rx_q_head = rx_q_tail = 0;  
    rx_dropped = 0;  
  
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
  
// Освобождает блок и перезапускает соответствующий DMA-канал.  
void rx_capture_release(void) {  
    const volatile uint32_t* buf = rx_queue[rx_q_tail];  
    rx_q_tail = (rx_q_tail + 1) % RX_QUEUE_DEPTH;  
    if (buf == rx_buf_a && rx_running) {  
        dma_channel_set_write_addr(rx_dma_a, (void*)rx_buf_a, RX_BLOCK_SAMPLES);  
        dma_channel_start(rx_dma_a);   // вручную — цепочка его уже прошла  
    } else if (buf == rx_buf_b && rx_running) {  
        dma_channel_set_write_addr(rx_dma_b, (void*)rx_buf_b, RX_BLOCK_SAMPLES);  
        dma_channel_start(rx_dma_b);  
    }  
}  
  
uint32_t rx_capture_dropped(void) { return rx_dropped; }