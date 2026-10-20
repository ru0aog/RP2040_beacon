/**
 * ============================================================================
 * rx_capture.cpp — DMA-движок приёмной части RP2040
 *
 * Архитектура:
 *   ADC round-robin: ADC0, ADC1, ADC0, ADC1, ...
 *   DMA A <-> DMA B: каналы запускают друг друга через chain_to.
 *   Core 0 / DMA IRQ0: публикует завершённые рабочие буферы в SPSC-очередь.
 *   Core 1 / DSP: получает блок, обрабатывает его и вызывает release.
 *
 * Формат:
 *   RX_BLOCK_SAMPLES — число 32-битных слов (пар I/Q) в блоке.
 *   Каждый 32-битный элемент заполняется двумя DMA-переносами по 16 бит:
 *     младшие 16 бит — первое преобразование ADC,
 *     старшие 16 бит — второе преобразование ADC.
 *   При round-robin с началом ADC0 ожидается порядок I,Q, если распиновка
 *   платы действительно связывает RX_ADC_PIN_I с ADC0, а RX_ADC_PIN_Q с ADC1.
 *
 * Владение памятью:
 *   - DMA пишет только в назначенный ему текущий буфер.
 *   - Опубликованный рабочий буфер помечается busy и не назначается DMA.
 *   - Core 1 снимает busy только после завершения чтения блока.
 *   - Если свободного рабочего буфера нет, DMA пишет в scratch; этот блок
 *     намеренно отбрасывается и никогда не публикуется потребителю.
 *
 * ВАЖНО О CHAIN_TO:
 *   Завершившийся канал автоматически запускает канал-напарник. IRQ должен
 *   успеть перенастроить завершившийся канал до того, как напарник закончит
 *   следующий блок и повторно запустит его. При блокировке IRQ дольше времени
 *   блока возможен повторный запуск с неготовыми регистрами. Эта архитектура
 *   требует короткого IRQ и не даёт абсолютной гарантии при произвольной
 *   задержке прерываний.
 *
 * API для rx_dsp.cpp:
 *   rx_capture_start()
 *   rx_capture_stop()
 *   rx_capture_get_block()
 *   rx_capture_release()
 *   rx_capture_dropped()
 * ============================================================================
 */

#include <Arduino.h>
#include <string.h>

#include "vfo_hardware.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// ---------------------------------------------------------------------------
// Параметры
// ---------------------------------------------------------------------------

/*
дефайны объявлены в vfo_hardware.h
#define RX_ADC_SAMPLE_RATE_HZ 250000UL // суммарная частота дискретизации (125 ksps на канал)
#define RX_BLOCK_SAMPLES      512         // отсчётов I/Q на блок (пар в 32-бит словах) 
#define RX_ADC_PIN_I          26          // GPIO26 = ADC0, канал I  
#define RX_ADC_PIN_Q          27          // GPIO27 = ADC1, канал Q  
*/

// Один блок содержит RX_BLOCK_SAMPLES пар I/Q, то есть вдвое больше
// 16-битных DMA-переносов, чем 32-битных слов-пар.
#define RX_BLOCK_TRANSFERS (2u * RX_BLOCK_SAMPLES)

// Кольцевая очередь использует один пустой слот для отличия full от empty.
// Поэтому при глубине 4 фактическая ёмкость очереди — 3 указателя.
#define RX_QUEUE_DEPTH 4u

#if RX_BLOCK_SAMPLES == 0
#error "RX_BLOCK_SAMPLES должен быть больше нуля"
#endif

// ---------------------------------------------------------------------------
// Буферы DMA
// ---------------------------------------------------------------------------

// Буферы выровнены по 4 байта. DMA работает переносами по 16 бит, но адреса
// и ёмкость буферов заданы в 32-битных словах для удобства DSP.
// volatile оставлен, поскольку данные меняются DMA независимо от CPU.
static volatile uint32_t __attribute__((aligned(4)))
    rx_buf_a0[RX_BLOCK_SAMPLES];
static volatile uint32_t __attribute__((aligned(4)))
    rx_buf_a1[RX_BLOCK_SAMPLES];
static volatile uint32_t __attribute__((aligned(4)))
    rx_buf_b0[RX_BLOCK_SAMPLES];
static volatile uint32_t __attribute__((aligned(4)))
    rx_buf_b1[RX_BLOCK_SAMPLES];

// Scratch-буферы принадлежат только соответствующим DMA-каналам.
// Они используются, когда оба рабочих буфера канала ещё заняты.
static volatile uint32_t __attribute__((aligned(4)))
    rx_scratch_a[RX_BLOCK_SAMPLES];
static volatile uint32_t __attribute__((aligned(4)))
    rx_scratch_b[RX_BLOCK_SAMPLES];

// ---------------------------------------------------------------------------
// SPSC-очередь завершённых блоков
// ---------------------------------------------------------------------------

// Пишет только DMA IRQ на Core 0, читает только DSP на Core 1.
// Публикация: сначала указатель, затем head.
// Освобождение: сначала DSP заканчивает чтение, затем продвигает tail.
static const volatile uint32_t* rx_queue[RX_QUEUE_DEPTH];
static volatile uint8_t rx_q_head = 0;  // producer: Core 0 / IRQ
static volatile uint8_t rx_q_tail = 0;  // consumer: Core 1 / DSP

// Общий счётчик потерь, совместимый с существующим API.
static volatile uint32_t rx_dropped = 0;

// Раздельные диагностические счётчики. Пока доступны внутри этого файла;
// при необходимости для них можно добавить отдельные getter-функции.
static volatile uint32_t rx_drop_scratch = 0;
static volatile uint32_t rx_drop_queue_full = 0;
static volatile uint32_t rx_drop_busy = 0;
static volatile uint32_t rx_irq_both_pending = 0;

// ---------------------------------------------------------------------------
// Владение рабочими буферами
// ---------------------------------------------------------------------------

// busy=true означает, что буфер опубликован DSP и ещё не освобождён.
// Изменения busy выполняются producer в IRQ и consumer в release.
// На RP2040 это одноядерные 8-битные значения; барьеры ниже задают порядок
// публикации данных и владения, но не превращают volatile в общий mutex.
static volatile bool rx_busy_a[2] = { false, false };
static volatile bool rx_busy_b[2] = { false, false };

// Подсказка, с какого буфера начинать поиск свободного рабочего буфера.
static volatile uint8_t rx_a_sel = 0;
static volatile uint8_t rx_b_sel = 0;

// Цель текущего/последнего настроенного переноса каждого DMA-канала.
// Эти значения обновляются только при старте и в DMA IRQ.
static volatile uint32_t* rx_dma_a_target = rx_buf_a0;
static volatile uint32_t* rx_dma_b_target = rx_buf_b0;

static int rx_dma_a = -1;
static int rx_dma_b = -1;
static volatile bool rx_running = false;

// ---------------------------------------------------------------------------
// Вспомогательные функции
// ---------------------------------------------------------------------------

static inline volatile uint32_t* rx_buffer_for(bool channel_a, int idx) {
    if (channel_a) {
        return (idx == 0) ? rx_buf_a0 : rx_buf_a1;
    }
    return (idx == 0) ? rx_buf_b0 : rx_buf_b1;
}

static inline volatile uint32_t* rx_scratch_for(bool channel_a) {
    return channel_a ? rx_scratch_a : rx_scratch_b;
}

static inline volatile bool* rx_busy_for(bool channel_a) {
    return channel_a ? rx_busy_a : rx_busy_b;
}

static inline volatile uint8_t* rx_select_for(bool channel_a) {
    return channel_a ? &rx_a_sel : &rx_b_sel;
}

// Возвращает индекс рабочего буфера или -1 для scratch/неизвестного адреса.
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
// Назначить следующую цель DMA и восстановить регистры переноса
// ---------------------------------------------------------------------------
// осуществляет динамическое перенаправление потока данных DMA «на лету»
// и восстанавливает конфигурационные регистры канала, который только что завершил запись блока.
// Она вызывается из прерывания rx_handle_completed_channel на ядре Core 0.
// Она должна подготовить канал к следующему старту в условиях, когда канал-напарник уже аппаратно запущен механизмом chain_to и вовсю оцифровывает эфир.
static void __not_in_flash_func(rx_rearm_channel)(int channel,bool channel_a) {
    // 1. Извлечение указателей состояния
    // Используя инлайн-хелперы, функция определяет, с каким каналом она работает (А или В),
    // и извлекает указатели на соответствующие флаги занятости буферов (rx_busy_X) и переменную-подсказку rx_X_sel.
    volatile bool* busy = rx_busy_for(channel_a);
    volatile uint8_t* sel = rx_select_for(channel_a);
    
    // 2. Алгоритм циклического поиска свободного буфера (Round-Robin)
    // Каждый канал DMA распоряжается парой собственных буферов (0 и 1).
    // Чтобы не тратить время на проверку буферов всегда с самого начала, код использует переменную *sel как стартовую точку.
    int chosen = -1;
    // Ищем первый свободный рабочий буфер, начиная с сохранённого выбора.
    // Как только программа находит буфер, у которого флаг busy == false (то есть ядро Core 1 / DSP его не удерживает и уже закончило обработку),
    // его индекс сохраняется в переменную chosen, а цикл досрочно завершается.
    for (int k = 0; k < 2; ++k) {
        uint8_t idx = (uint8_t)((*sel + (uint8_t)k) & 1u);
        if (!busy[idx]) {
            chosen = (int)idx;
            break;
        }
    }

    // 3. Динамическое ветвление: Рабочий буфер vs Scratch
    volatile uint32_t* target;
    // Сценарий Оптимистичный (chosen >= 0): Найдена свободная рабочая область. 
    // В переменную target записывается физический адрес этого буфера (например, rx_buf_a1). 
    // Переменная *sel инвертируется с помощью XOR (chosen ^ 1), чтобы на следующем круге прерывание сразу начинало поиск со второго буфера пары.
    if (chosen >= 0) {
        target = rx_buffer_for(channel_a, chosen);
        *sel = (uint8_t)(chosen ^ 1);
    } else {
    // Сценарий Аварийный (chosen == -1): Сигнальный процессор (Core 1) перегружен математикой
    // и не успел вызвать release() ни для одного из двух буферов текущего канала. 
    // Чтобы не затереть полезные данные, которые DSP прямо сейчас обсчитывает, DMA перенаправляется на запись в резервный буфер-черновик rx_scratch_X. 
    // Данные оцифровки АЦП продолжают записываться, синусоида не рвется, но этот блок на следующем круге будет отброшен.
        target = rx_scratch_for(channel_a);     // Оба рабочих буфера заняты потребителем: не перезаписываем их
    }

    // 4. Фиксация цели записи
    // сохраняет вычисленный адрес назначения в глобальный указатель rx_dma_X_target
    if (channel_a) {
        rx_dma_a_target = target;
    } else {
        rx_dma_b_target = target;
    }

    // 5. Восстановление аппаратных регистров контроллера DMA
    // false означает: настроить регистры, но не запускать канал программно.
    dma_channel_set_write_addr(channel, (void*)target, false);          // Записывает новый целевой адрес target в регистр WRITE_ADDR канала DMA
    dma_channel_set_trans_count(channel, RX_BLOCK_TRANSFERS, false);    // Прошивает количество 16-битных переносов RX_BLOCK_TRANSFERS (1024 децибела) в регистр TRANS_COUNT
    // последний параметр указывает, нужно ли вызывать немедленный программный старт канала. Код передает false. Канал просто перенастраивается, взводится на боевой взвод и засыпает.
    // Он запустится полностью аппаратно, без участия софта, ровно тогда, когда канал-напарник закончит молотить свой блок и выдаст ему пинок через встроенный в кремний механизм chain_to.
}

// ---------------------------------------------------------------------------
// Обработка одного завершившегося канала
// ---------------------------------------------------------------------------
// забирает у контроллера DMA только что заполненный блок сырых данных АЦП, 
// передаёт его через межъядерный Lock-Free шлюз для обработки на Core 1
// перенастраивает канал на следующий круг
static void __not_in_flash_func(rx_handle_completed_channel)(int channel, bool channel_a) {

    // 1. Аппаратный сброс IRQ-флага этого DMA-канала и проверка статуса
    dma_channel_acknowledge_irq0(channel);
    // Если глобально была вызвана команда остановки приёма, функция мгновенно выходит, предотвращая дальнейшую публикацию блоков и перезапуск конвейера.
    if (!rx_running) {
        return;
    }
    
    // 2. Извлечение адреса и определение логического индекса
    // считывает указатель rx_dma_X_target — адрес той области памяти,
    // куда завершившийся канал DMA физически записывал данные на протяжении последнего блока
    const volatile uint32_t* completed_target =
        channel_a ? rx_dma_a_target : rx_dma_b_target;
    // rx_find_buffer сопоставляет этот адрес с массивами в ОЗУ и возвращает логический индекс рабочего буфера:
    // 0 (для buf_X0) или 1 (для buf_X1).
    const int idx = rx_find_buffer(completed_target, channel_a);

    // 3. Защита 1: Фиксация работы вхолостую (Scratch)
    if (idx < 0) {
        // Если rx_find_buffer вернула значение < 0, это означает, что на предыдущем шаге
        // из-за перегрузки сигнального процессора (Core 1) канал DMA был вынужден сливать ВЧ-поток в резервный буфер-черновик rx_scratch_X.
        // Блок ушёл в scratch: он намеренно отбрасывается.
        ++rx_dropped;       // счётчик потерь
        ++rx_drop_scratch;  // из-за слива буфера
        rx_rearm_channel(channel, channel_a);
        return;
    }

    // 4. Защита 2: Проверка свободного места в межъядерной очереди
    volatile bool* busy = rx_busy_for(channel_a);
    // Проверяем наличие места до публикации рабочего буфера.
    const uint8_t head = rx_q_head;
    const uint8_t tail = rx_q_tail;
    const uint8_t next = (uint8_t)((head + 1u) % RX_QUEUE_DEPTH);
    // если next == tail, это означает, что кольцевая Lock-Free очередь полностью забита
    // (все её 3 рабочих слота заняты указателями блоков, которые Core 1 ещё не успело переварить
    if (next == tail) {
        // Очередь полна. Не публикуем указатель и не помечаем этот блок busy.
        ++rx_dropped;           // счётчик потерь
        ++rx_drop_queue_full;   // из-за переполнения очереди
        rx_rearm_channel(channel, channel_a);
        return;
    }
    
    // 5. Защита 3: Проверка протокола владения (Сбой флагов)
    // Если очередь рассинхронизировалась и буфер, который DMA только что закончило заполнять,
    // всё ещё имеет статус busy == true (то есть Core 1 прямо сейчас его читает), публикация блокируется
    if (busy[idx]) {
        // Такой сценарий не должен возникать, если повторное назначение
        // буферов и release соблюдают протокол владения.
        ++rx_dropped;           // счётчик потерь
        ++rx_drop_busy;         // из-за занятости буфера
        rx_rearm_channel(channel, channel_a);
        return;
    }

    // 6. Потокобезопасная Lock-Free публикация блока в ОЗУ
    // Текущий буфер официально резервируется и переходит во владение ядра обработки.
    // До тех пор, пока Core 1 не снимет этот флаг в функции release, драйвер захвата больше не отдаст этот массив контроллеру DMA
    busy[idx] = true;
    // флаг busy физически обновится в ячейках оперативной памяти до того, как адрес буфера появится в очереди
    __dmb();
    // Физический адрес заполненного массива ОЗУ записывается в текущий пустой слот очереди
    // Данные буфера никуда не копируются, передаётся только 32-битный указатель, что даёт максимальное быстродействие
    rx_queue[head] = completed_target;
    // указатель гарантированно ложится на шину памяти до того, как сдвинется индекс готовности.
    __dmb();
    // Единственная точка публикации готового блока для Core 1.
    // Как только индекс сдвигается, ядро Core 1 в своём цикле ожидания rx_dsp_entry мгновенно видит новый блок и забирает его на фильтрацию.
    rx_q_head = next;
    
    // 7. Перенастройка на ходу
    // Немедленно подготавливаем этот DMA-канал к следующему циклу.
    rx_rearm_channel(channel, channel_a);
}

// ---------------------------------------------------------------------------
// IRQ DMA0
// ---------------------------------------------------------------------------
// Короткий ISR: главный обработчик прерываний (ISR — Interrupt Service Routine) для всего приёмного тракта.
// Выполняется на Core 0 по аппаратному вектору прерывания DMA_IRQ_0 в тот самый момент, 
// когда один из каналов DMA завершает заполнение своего блока данных.
static void __not_in_flash_func(rx_dma_irq)(void) {
    // Локальное кэширование индексов каналов
    // Считываем динамические номера аппаратных каналов DMA, которые выделила система при старте,
    // и сохраняем их в локальные неизменяемые (const) переменные. 
    const int a = rx_dma_a;
    const int b = rx_dma_b;
    // Опрос аппаратных флагов прерывания (Инспекция регистров)
    // Прерывание DMA_IRQ_0 является общим для всех каналов DMA в RP2040. Процессор не знает, какой именно из каналов вызвал прерывание.
    // Смотрим в системный аппаратный регистр DMA_INTS0 контроллера DMA и проверяем конкретные биты, соответствующие каналам a и b.
    // Если бит равен 1 — значит, именно этот канал закончил перенос и требует обслуживания процессором.
    const bool a_pending =
        (a >= 0) && dma_channel_get_irq0_status(a);
    const bool b_pending =
        (b >= 0) && dma_channel_get_irq0_status(b);
    // Диагностика критического джиттера
    // Ситуация, когда оба флага a_pending и b_pending одновременно равны true, 
    // в идеальном приемнике физически невозможна, так как каналы работают по очереди по схеме Пинг-Понг
    if (a_pending && b_pending) {
        ++rx_irq_both_pending;  // ядро Core 0 было заблокировано каким-то процессом на недопустимо огромное время
    }
    // Селективное диспетчеризирование каналов
    if (a_pending) { rx_handle_completed_channel(a, true);  }
    if (b_pending) { rx_handle_completed_channel(b, false); }
}

// ---------------------------------------------------------------------------
// Запуск захвата
// ---------------------------------------------------------------------------
void rx_capture_start(void) {
    if (rx_running) { return; }

    // 1. Сброс программного состояния и подготовка диагностических метрик
        // межъядерная Lock-Free очередь
        rx_q_head = 0;
        rx_q_tail = 0;
        // счетчики группы потерь
        rx_dropped = 0;         // всего потерь
        rx_drop_scratch = 0;    // слив в мусорный буфер
        rx_drop_queue_full = 0; // забита очередь
        rx_drop_busy = 0;       // буфер не успели освободить
        rx_irq_both_pending = 0;
        // четыре рабочих буфера
        rx_busy_a[0] = rx_busy_a[1] = false;
        rx_busy_b[0] = rx_busy_b[1] = false;
        // индексы выбора буферов
        rx_a_sel = 0;
        rx_b_sel = 0;
        // начальные указатели
        rx_dma_a_target = rx_buf_a0;
        rx_dma_b_target = rx_buf_b0;

    // 2. Конфигурация аппаратного блока АЦП
        // Включается аппаратный блок АЦП (тактируется от периферийной шины 48 МГц)
        // соответствующие ножки микроконтроллера переключаются в аналоговый режим
        adc_init();
        adc_gpio_init(RX_ADC_PIN_I);  // GPIO26 = ADC0, канал I  
        adc_gpio_init(RX_ADC_PIN_Q);  // GPIO27 = ADC1, канал Q
        // Квадратурный Round-Robin
        // Биты 0 и 1: ADC0 и ADC1. Ожидаемый порядок — 0,1,0,1,...
        adc_set_round_robin(0b00011);
        // Режим FIFO и аппаратные запросы (DREQ)
        // FIFO включён; DMA DREQ включён; порог 1 элемент.
        // FIFO RP2040 имеет глубину 4 элемента, поэтому threshold должен быть 1..4.
        adc_fifo_setup(
            true,   // включить FIFO
            true,   // включить DREQ для DMA
            1,      // DREQ threshold
            false,  // не добавлять error bit в FIFO
            false   // не сдвигать результат до 8 бит
        );

    // 3. Расчёт прецизионного тактирования АЦП
        // RX_ADC_SAMPLE_RATE_HZ трактуется как суммарная частота преобразований
        // ADC0+ADC1. Частота одного канала при чередовании вдвое меньше.
        // Формула соответствует модели SDK: Fs = clk_adc / (1 + clkdiv).
        float adc_div =
            48000000.0f / (float)RX_ADC_SAMPLE_RATE_HZ - 1.0f;
        // защита от отрицательных значений
        if (adc_div < 0.0f) {
            adc_div = 0.0f;
        }
        // установить делитель шины периферии для тактирования АЦП
        adc_set_clkdiv(adc_div);

    // 4. Развертывание Пинг-Понг конвейера DMA (Hardware Chaining)
        // Получаем два DMA-канала. Если свободных каналов нет, SDK при параметре
        // true обычно аварийно завершает выполнение; в проекте предполагается,
        // что два канала доступны. При необходимости это место можно заменить
        // на явную проверку dma_claim_unused_channel(false).
        rx_dma_a = dma_claim_unused_channel(true);
        rx_dma_b = dma_claim_unused_channel(true);

        // Конфигурация канала A: FIFO -> A0; после завершения запускает B.
        dma_channel_config ca = dma_channel_get_default_config(rx_dma_a);
        channel_config_set_transfer_data_size(&ca, DMA_SIZE_16);
        channel_config_set_read_increment(&ca, false);
        channel_config_set_write_increment(&ca, true);
        channel_config_set_dreq(&ca, DREQ_ADC);
        channel_config_set_chain_to(&ca, rx_dma_b);

        // Конфигурация канала B: FIFO -> B0; после завершения запускает A.
        dma_channel_config cb = dma_channel_get_default_config(rx_dma_b);
        channel_config_set_transfer_data_size(&cb, DMA_SIZE_16);
        channel_config_set_read_increment(&cb, false);
        channel_config_set_write_increment(&cb, true);
        channel_config_set_dreq(&cb, DREQ_ADC);
        channel_config_set_chain_to(&cb, rx_dma_a);

        dma_channel_configure(
            rx_dma_a, &ca, (void*)rx_buf_a0, &adc_hw->fifo,
            RX_BLOCK_TRANSFERS, false
        );
        dma_channel_configure(
            rx_dma_b, &cb, (void*)rx_buf_b0, &adc_hw->fifo,
            RX_BLOCK_TRANSFERS, false
        );

        rx_dma_a_target = rx_buf_a0;
        rx_dma_b_target = rx_buf_b0;

    // 5. Конфигурация вектора прерываний (NVIC)
        // Удаляем старые IRQ-флаги и разрешаем IRQ для этих каналов.
        dma_channel_acknowledge_irq0(rx_dma_a);
        dma_channel_acknowledge_irq0(rx_dma_b);
        dma_channel_set_irq0_enabled(rx_dma_a, true);
        dma_channel_set_irq0_enabled(rx_dma_b, true);

        // Эта версия предполагает, что DMA_IRQ_0 целиком принадлежит rx_capture.
        // Если другие модули используют тот же IRQ, нужен общий диспетчер IRQ.
        irq_set_exclusive_handler(DMA_IRQ_0, rx_dma_irq);
    // 6. Аппаратная очистка и синхронизированный пуск
        // До старта DMA останавливаем ADC и очищаем FIFO от старых значений.
        adc_run(false);
        adc_fifo_drain();
        adc_select_input(0);

        // Ставим флаг до запуска DMA/ADC, чтобы IRQ видел согласованное состояние.
        rx_running = true;
        irq_set_enabled(DMA_IRQ_0, true);

        // Сначала готовим DMA A, затем запускаем ADC. После завершения A аппаратно
        // стартует B, затем A и так далее.
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

// Контракт вызова: сначала остановить DSP-потребитель (rx_dsp_stop()), затем
// вызвать rx_capture_stop(). Так Core 1 не продолжит читать очередь/буфер,
// пока этот модуль сбрасывает состояние и останавливает DMA.
void rx_capture_stop(void) {
    // Сначала запрещаем публикацию новых блоков обработчиком IRQ.
    rx_running = false;

    // Останавливаем источник новых ADC-преобразований.
    adc_run(false);

    // Маскируем общий IRQ. Это безопасно, только если DMA_IRQ_0 принадлежит
    // исключительно этому модулю.
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

    // Очередь очищается только после остановки DSP-потребителя.
    rx_q_head = 0;
    rx_q_tail = 0;

    rx_busy_a[0] = rx_busy_a[1] = false;
    rx_busy_b[0] = rx_busy_b[1] = false;
    rx_a_sel = 0;
    rx_b_sel = 0;

    rx_dma_a_target = rx_buf_a0;
    rx_dma_b_target = rx_buf_b0;

    // Возвращаем DMA-каналы системе для повторного использования.
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
// API очереди — вызывается DSP-потребителем на Core 1
// ---------------------------------------------------------------------------

// Возвращённый указатель действителен до rx_capture_release().
// После успешного get должен быть ровно один release для этого блока.
const volatile uint32_t* rx_capture_get_block(void) {
    const uint8_t tail = rx_q_tail;

    if (tail == rx_q_head) {
        return NULL;
    }

    // Producer записал указатель перед публикацией head. Барьер не даёт
    // последующим чтениям блока переместиться перед проверкой очереди.
    __dmb();
    return rx_queue[tail];
}

// Освободить текущий блок после завершения обработки DSP.
void rx_capture_release(void) {
    const uint8_t tail = rx_q_tail;

    // Защита от release без успешного get или повторного release.
    if (tail == rx_q_head) {
        return;
    }

    const volatile uint32_t* buf = rx_queue[tail];

    // Все чтения DSP должны закончиться до освобождения слота/буфера.
    __dmb();

    // Сначала освобождаем слот очереди.
    rx_q_tail = (uint8_t)((tail + 1u) % RX_QUEUE_DEPTH);

    // Затем разрешаем DMA повторно использовать рабочий буфер.
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
// Диагностика
// ---------------------------------------------------------------------------

// Общий счётчик потерь для совместимости с текущим rx_dsp.cpp.
// Суммирует scratch, переполнение очереди и неожиданное состояние busy.
uint32_t rx_capture_dropped(void) {
    return rx_dropped;
}
