
/**
 * ============================================================================
 * rx_capture.cpp — DMA-движок приёмной части RP2040
 *
 * Два DMA-канала A/B с аппаратным chain_to.
 *
 * Каждый канал имеет два рабочих буфера и собственный scratch:
 *   A: rx_buf_a0, rx_buf_a1, rx_scratch_a
 *   B: rx_buf_b0, rx_buf_b1, rx_scratch_b
 *
 * ADC0 и ADC1 работают по кругу. DMA читает FIFO 16-битными переносами
 * и записывает последовательность [I,Q,I,Q,...] в память.
 *
 * RX_BLOCK_SAMPLES — количество пар I/Q в одном блоке.
 * RX_ADC_SAMPLE_RATE_HZ — суммарная частота преобразований ADC.
 *
 * Важно:
 *   - DMA-канал после завершения необходимо перенастроить: write_addr
 *     и trans_count не восстанавливаются автоматически.
 *   - Рабочий буфер не используется повторно, пока его не освободит DSP.
 *   - Scratch никогда не публикуется в очереди.
 *   - Очередь SPSC: IRQ на core0 производит, DSP на core1 потребляет.
 * ============================================================================
 */

#include <Arduino.h>

#include "vfo_hardware.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// ---------------------------------------------------------------------------
// Константы
// ---------------------------------------------------------------------------

// Количество 16-битных DMA-переносов в одном блоке.
// Один блок RX_BLOCK_SAMPLES содержит RX_BLOCK_SAMPLES пар I/Q.
#define RX_BLOCK_TRANSFERS (2u * RX_BLOCK_SAMPLES)

// Глубина очереди. Один слот оставляем пустым для отличия full от empty.
#define RX_QUEUE_DEPTH 4

// ---------------------------------------------------------------------------
// Буферы
// ---------------------------------------------------------------------------

static volatile uint32_t __attribute__((aligned(4)))
    rx_buf_a0[RX_BLOCK_SAMPLES];

static volatile uint32_t __attribute__((aligned(4)))
    rx_buf_a1[RX_BLOCK_SAMPLES];

static volatile uint32_t __attribute__((aligned(4)))
    rx_buf_b0[RX_BLOCK_SAMPLES];

static volatile uint32_t __attribute__((aligned(4)))
    rx_buf_b1[RX_BLOCK_SAMPLES];

static volatile uint32_t __attribute__((aligned(4)))
    rx_scratch_a[RX_BLOCK_SAMPLES];

static volatile uint32_t __attribute__((aligned(4)))
    rx_scratch_b[RX_BLOCK_SAMPLES];

// ---------------------------------------------------------------------------
// Очередь готовых блоков
// ---------------------------------------------------------------------------

static const volatile uint32_t* rx_queue[RX_QUEUE_DEPTH];

static volatile uint8_t rx_q_head = 0;  // пишет core0, DMA IRQ
static volatile uint8_t rx_q_tail = 0;  // пишет core1, DSP

static volatile uint32_t rx_dropped = 0;

// ---------------------------------------------------------------------------
// Состояние рабочих буферов
//
// busy[i] = true означает, что буфер опубликован в очереди и ещё не
// освобождён потребителем.
//
// *_sel — индекс, с которого начинать поиск следующего свободного буфера.
// ---------------------------------------------------------------------------

static volatile bool rx_busy_a[2] = { false, false };
static volatile bool rx_busy_b[2] = { false, false };

static volatile uint8_t rx_a_sel = 0;
static volatile uint8_t rx_b_sel = 0;

// Куда записывает каждый DMA-канал в текущем переносе.
// Эти указатели меняются только при перенастройке канала.
static volatile uint32_t* rx_dma_a_target = rx_buf_a0;
static volatile uint32_t* rx_dma_b_target = rx_buf_b0;

static int rx_dma_a = -1;
static int rx_dma_b = -1;

static volatile bool rx_running = false;

// ---------------------------------------------------------------------------
// Найти индекс рабочего буфера по адресу.
//
// Возвращает:
//   0 — первый рабочий буфер
//   1 — второй рабочий буфер
//  -1 — scratch или неизвестный адрес
// ---------------------------------------------------------------------------

static int __not_in_flash_func(rx_find_buffer)(
    const volatile uint32_t* target,
    bool channel_a
) {
    if (channel_a) {
        if (target == rx_buf_a0) return 0;
        if (target == rx_buf_a1) return 1;
    } else {
        if (target == rx_buf_b0) return 0;
        if (target == rx_buf_b1) return 1;
    }

    return -1;
}

// ---------------------------------------------------------------------------
// Выбрать назначение следующего переноса и перенастроить DMA.
//
// Выбирается первый незанятый рабочий буфер, начиная с *_sel.
// Если оба буфера заняты, используется scratch соответствующего канала.
//
// Вызывается из IRQ после обработки завершившегося переноса.
// ---------------------------------------------------------------------------

static void __not_in_flash_func(rx_rearm_channel)(
    int channel,
    bool channel_a
) {
    volatile uint32_t* b0 =
        channel_a ? rx_buf_a0 : rx_buf_b0;

    volatile uint32_t* b1 =
        channel_a ? rx_buf_a1 : rx_buf_b1;

    volatile uint32_t* scratch =
        channel_a ? rx_scratch_a : rx_scratch_b;

    volatile bool* busy =
        channel_a ? rx_busy_a : rx_busy_b;

    volatile uint8_t* sel =
        channel_a ? &rx_a_sel : &rx_b_sel;

    int chosen = -1;

    // Ищем свободный рабочий буфер.
    for (int k = 0; k < 2; ++k) {
        uint8_t idx = (uint8_t)((*sel + k) & 1u);

        if (!busy[idx]) {
            chosen = idx;
            break;
        }
    }

    volatile uint32_t* target;

    if (chosen == 0) {
        target = b0;
        *sel = 1;
    } else if (chosen == 1) {
        target = b1;
        *sel = 0;
    } else {
        // Оба рабочих буфера ещё используются.
        target = scratch;
    }

    if (channel_a) {
        rx_dma_a_target = target;
    } else {
        rx_dma_b_target = target;
    }

    // После завершения переноса счётчик равен нулю.
    // Восстанавливаем адрес и размер до следующего запуска канала.
    dma_channel_set_write_addr(
        channel,
        (void*)target,
        false
    );

    dma_channel_set_trans_count(
        channel,
        RX_BLOCK_TRANSFERS,
        false
    );
}

// ---------------------------------------------------------------------------
// Обработать завершение одного DMA-канала.
//
// chain_to уже запустил канал-напарник. Поэтому текущий канал нужно
// перенастроить до того, как напарник завершит свой блок и запустит его снова.
// ---------------------------------------------------------------------------

static void __not_in_flash_func(rx_handle_completed_channel)(
    int channel,
    bool channel_a
) {
    dma_channel_acknowledge_irq0(channel);

    if (!rx_running) {
        return;
    }

    // Сохраняем назначение именно завершившегося переноса.
    const volatile uint32_t* completed_target =
        channel_a ? rx_dma_a_target : rx_dma_b_target;

    int idx = rx_find_buffer(completed_target, channel_a);

    if (idx < 0) {
        // Перенос был записан в scratch: блок намеренно теряется.
        rx_dropped++;
        rx_rearm_channel(channel, channel_a);
        return;
    }

    volatile bool* busy =
        channel_a ? rx_busy_a : rx_busy_b;

    // Проверяем очередь до резервирования буфера.
    uint8_t head = rx_q_head;
    uint8_t next = (uint8_t)((head + 1u) % RX_QUEUE_DEPTH);

    if (next == rx_q_tail || busy[idx]) {
        // Очередь заполнена или буфер неожиданно уже занят.
        // Не публикуем этот блок и не устанавливаем busy.
        rx_dropped++;
        rx_rearm_channel(channel, channel_a);
        return;
    }

    // Резервируем рабочий буфер до публикации указателя.
    busy[idx] = true;
    __dmb();

    rx_queue[head] = completed_target;

    // Указатель в очереди должен быть записан до публикации нового head.
    __dmb();
    rx_q_head = next;

    // Теперь выбирается другой свободный буфер, если он есть.
    // Если оба буфера заняты, следующий перенос пойдёт в scratch.
    rx_rearm_channel(channel, channel_a);
}

// ---------------------------------------------------------------------------
// DMA IRQ0
// ---------------------------------------------------------------------------

static void __not_in_flash_func(rx_dma_irq)(void) {
    if (rx_dma_a >= 0 &&
        dma_channel_get_irq0_status(rx_dma_a)) {
        rx_handle_completed_channel(rx_dma_a, true);
    }

    if (rx_dma_b >= 0 &&
        dma_channel_get_irq0_status(rx_dma_b)) {
        rx_handle_completed_channel(rx_dma_b, false);
    }
}

// ---------------------------------------------------------------------------
// Запуск захвата
// ---------------------------------------------------------------------------

void rx_capture_start(void) {
    if (rx_running) {
        return;
    }

    rx_q_head = 0;
    rx_q_tail = 0;
    rx_dropped = 0;

    rx_busy_a[0] = false;
    rx_busy_a[1] = false;
    rx_busy_b[0] = false;
    rx_busy_b[1] = false;

    rx_a_sel = 0;
    rx_b_sel = 0;

    rx_dma_a_target = rx_buf_a0;
    rx_dma_b_target = rx_buf_b0;

    // ADC.
    adc_init();

    adc_gpio_init(RX_ADC_PIN_I);
    adc_gpio_init(RX_ADC_PIN_Q);

    // ADC0 и ADC1 по кругу.
    adc_set_round_robin(0b00011);

    // FIFO: включён, DREQ включён, порог 1 элемент.
    // FIFO RP2040 имеет глубину 4 элемента; порог 8 недопустим.
    adc_fifo_setup(
        true,   // FIFO enable
        true,   // DREQ enable
        1,      // DREQ threshold: 1..4
        false,  // error bit in FIFO
        false   // shift to 8 bits
    );

    // SDK RP2040 использует Fs = clk_adc / (1 + clkdiv).
    float adc_div =
        48000000.0f / (float)RX_ADC_SAMPLE_RATE_HZ - 1.0f;

    if (adc_div < 0.0f) {
        adc_div = 0.0f;
    }

    adc_set_clkdiv(adc_div);

    // Захватываем DMA-каналы.
    rx_dma_a = dma_claim_unused_channel(true);
    rx_dma_b = dma_claim_unused_channel(true);

    // Канал A: FIFO -> рабочий буфер A0; по завершении запускает B.
    dma_channel_config ca =
        dma_channel_get_default_config(rx_dma_a);

    channel_config_set_transfer_data_size(&ca, DMA_SIZE_16);
    channel_config_set_read_increment(&ca, false);
    channel_config_set_write_increment(&ca, true);
    channel_config_set_dreq(&ca, DREQ_ADC);
    channel_config_set_chain_to(&ca, rx_dma_b);

    // Канал B: FIFO -> рабочий буфер B0; по завершении запускает A.
    dma_channel_config cb =
        dma_channel_get_default_config(rx_dma_b);

    channel_config_set_transfer_data_size(&cb, DMA_SIZE_16);
    channel_config_set_read_increment(&cb, false);
    channel_config_set_write_increment(&cb, true);
    channel_config_set_dreq(&cb, DREQ_ADC);
    channel_config_set_chain_to(&cb, rx_dma_a);

    dma_channel_configure(
        rx_dma_a,
        &ca,
        (void*)rx_buf_a0,
        &adc_hw->fifo,
        RX_BLOCK_TRANSFERS,
        false
    );

    dma_channel_configure(
        rx_dma_b,
        &cb,
        (void*)rx_buf_b0,
        &adc_hw->fifo,
        RX_BLOCK_TRANSFERS,
        false
    );

    rx_dma_a_target = rx_buf_a0;
    rx_dma_b_target = rx_buf_b0;

    // Сброс старых флагов IRQ.
    dma_channel_acknowledge_irq0(rx_dma_a);
    dma_channel_acknowledge_irq0(rx_dma_b);

    dma_channel_set_irq0_enabled(rx_dma_a, true);
    dma_channel_set_irq0_enabled(rx_dma_b, true);

    irq_set_exclusive_handler(DMA_IRQ_0, rx_dma_irq);

    // Останавливаем ADC, очищаем FIFO и выбираем стартовый вход.
    adc_run(false);
    adc_fifo_drain();
    adc_select_input(0);

    rx_running = true;

    irq_set_enabled(DMA_IRQ_0, true);

    // Сначала готовим DMA, затем запускаем преобразования ADC.
    dma_channel_start(rx_dma_a);
    adc_run(true);

    Serial.printf(
        "[CAP] chA=%d chB=%d block=%u pairs transfers=%u\n",
        rx_dma_a,
        rx_dma_b,
        (unsigned)RX_BLOCK_SAMPLES,
        (unsigned)RX_BLOCK_TRANSFERS
    );

    Serial.printf(
        "[CAP] ADC conversions target=%lu/s clkdiv=%.3f\n",
        (unsigned long)RX_ADC_SAMPLE_RATE_HZ,
        adc_div
    );
}

// ---------------------------------------------------------------------------
// Остановка захвата
// ---------------------------------------------------------------------------

void rx_capture_stop(void) {
    // Вызов должен происходить в согласованной точке жизненного цикла:
    // DSP не должен продолжать читать буфер после остановки и сброса очереди.
    rx_running = false;

    adc_run(false);

    // Запрещаем новые IRQ и останавливаем DMA.
    irq_set_enabled(DMA_IRQ_0, false);

    if (rx_dma_a >= 0) {
        dma_channel_set_irq0_enabled(rx_dma_a, false);
        dma_channel_abort(rx_dma_a);
        dma_channel_acknowledge_irq0(rx_dma_a);
    }

    if (rx_dma_b >= 0) {
        dma_channel_set_irq0_enabled(rx_dma_b, false);
        dma_channel_abort(rx_dma_b);
        dma_channel_acknowledge_irq0(rx_dma_b);
    }

    adc_fifo_drain();

    rx_q_head = 0;
    rx_q_tail = 0;

    rx_busy_a[0] = false;
    rx_busy_a[1] = false;
    rx_busy_b[0] = false;
    rx_busy_b[1] = false;

    rx_a_sel = 0;
    rx_b_sel = 0;

    rx_dma_a_target = rx_buf_a0;
    rx_dma_b_target = rx_buf_b0;

    // Освобождаем каналы для повторного запуска.
    if (rx_dma_a >= 0) {
        dma_channel_unclaim(rx_dma_a);
        rx_dma_a = -1;
    }

    if (rx_dma_b >= 0) {
        dma_channel_unclaim(rx_dma_b);
        rx_dma_b = -1;
    }
}

// ---------------------------------------------------------------------------
// Интерфейс DSP — core1
//
// Полученный указатель действителен до rx_capture_release().
// Не вызывать release без успешного get.
// ---------------------------------------------------------------------------

const volatile uint32_t* rx_capture_get_block(void) {
    uint8_t tail = rx_q_tail;

    if (tail == rx_q_head) {
        return NULL;
    }

    // Сначала producer записывает указатель, затем публикует head.
    __dmb();

    return rx_queue[tail];
}

// ---------------------------------------------------------------------------
// Освободить блок после завершения обработки DSP.
// ---------------------------------------------------------------------------

void rx_capture_release(void) {
    uint8_t tail = rx_q_tail;

    if (tail == rx_q_head) {
        return;
    }

    const volatile uint32_t* buf = rx_queue[tail];

    // Все чтения блока должны завершиться до продвижения tail и снятия busy.
    __dmb();

    rx_q_tail = (uint8_t)((tail + 1u) % RX_QUEUE_DEPTH);

    __dmb();

    if (buf == rx_buf_a0) {
        rx_busy_a[0] = false;
    } else if (buf == rx_buf_a1) {
        rx_busy_a[1] = false;
    } else if (buf == rx_buf_b0) {
        rx_busy_b[0] = false;
    } else if (buf == rx_buf_b1) {
        rx_busy_b[1] = false;
    }
}

// ---------------------------------------------------------------------------
// Количество отброшенных блоков.
// Учитывает записи в scratch, переполнение очереди и неожиданный busy.
// ---------------------------------------------------------------------------

uint32_t rx_capture_dropped(void) {
    return rx_dropped;
}
