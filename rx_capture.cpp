#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "hardware/adc.h"  
#include "hardware/dma.h"  
#include "hardware/irq.h"  

// Пины АЦП: I-канал GP26 (ADC0), Q-канал GP27 (ADC1)  
#define RX_ADC_PIN_I          26  
#define RX_ADC_PIN_Q          27  
  
// Скорость потокового захвата, отсчётов/с  
#define RX_ADC_SAMPLE_RATE_HZ 480000.0f   // 480 ksps — потолок SAR при 48 МГц клока
  
// Буферы: 2 канала × 256 отсчётов × 2 (ping-pong)  
#define RX_BLOCK_SAMPLES   256  
static uint16_t rx_buf[2][RX_BLOCK_SAMPLES * 2];  
static volatile uint8_t  rx_ready_idx = 0xFF;   // 0xFF = нет готового блока  
static int rx_dma_chan = -1;  
  
// Cбор готового блока из loop(): {I,Q} чередуются, round-robin ADC  
bool rx_capture_get_block(const uint16_t** buf) {  
    if (rx_ready_idx == 0xFF) return false;  
    *buf = rx_buf[rx_ready_idx];  
    return true;  
}  
void rx_capture_release(void) { rx_ready_idx = 0xFF; }  
  
static void __not_in_flash_func(rx_dma_handler)(void) {  
    uint32_t ch = rx_dma_chan;  
    if (dma_channel_get_irq1_status(ch)) {  
        dma_channel_acknowledge_irq1(ch);  
        uint8_t done = (dma_hw->ch[ch].write_addr == (uintptr_t)&rx_buf[0][RX_BLOCK_SAMPLES * 2]) ? 1 : 0;  
        // Перезапуск на другую половину  
        dma_channel_set_write_addr(ch, rx_buf[done ^ 1], true);  
        rx_ready_idx = done;  
    }  
}  
  
void rx_capture_start(void) {  
    adc_init();  
    adc_gpio_init(RX_ADC_PIN_I);  
    adc_gpio_init(RX_ADC_PIN_Q);  
    adc_set_round_robin(0b00011);            // ADC0 + ADC1 по кругу  
    adc_fifo_setup(true, true, 1, false, false); // FIFO → DREQ, без сдвига (12 бит в младших)  
    adc_set_clkdiv(48000000.0f / RX_ADC_SAMPLE_RATE_HZ);  // целевая скорость  
  
    rx_dma_chan = dma_claim_unused_channel(true);  
    dma_channel_config dc = dma_channel_get_default_config(rx_dma_chan);  
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_16);  
    channel_config_set_read_increment(&dc, false);  
    channel_config_set_write_increment(&dc, true);  
    channel_config_set_dreq(&dc, DREQ_ADC);  
    channel_config_set_chain_to(&dc, rx_dma_chan);  // самоперезапуск  
  
    dma_channel_configure(rx_dma_chan, &dc, rx_buf[0],  
                          &adc_hw->fifo, RX_BLOCK_SAMPLES * 2, false);  
    dma_channel_acknowledge_irq1(rx_dma_chan);  
    dma_channel_set_irq1_enabled(rx_dma_chan, true);  
    irq_set_exclusive_handler(DMA_IRQ_1, rx_dma_handler);  
    irq_set_enabled(DMA_IRQ_1, true);  
  
    adc_fifo_drain();  
    adc_run(true);  
    dma_channel_start(rx_dma_chan);  
}  
  
void rx_capture_stop(void) {  
    adc_run(false);  
    dma_channel_abort(rx_dma_chan);  
    dma_channel_set_irq1_enabled(rx_dma_chan, false);  
    adc_fifo_drain();  
}