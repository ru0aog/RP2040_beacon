#include "vfo_hardware.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/resets.h"
#include "hardware/timer.h"

// Ассемблерная микропрограмма PIO для меандра (цикл из 2 тактов): set pins 1 -> set pins 0
static const uint16_t pio_square_instructions[] = { 0xe001, 0xe000 };
static const pio_program_t pio_square_program = { .instructions = pio_square_instructions, .length = 2, .origin = -1 };

// Выделение памяти под глобальные дескрипторы и переменные дизеринга
VfoParameters ifkp_tones[VFO_IFKP_TONES_COUNT];
static PIO lo_pio = pio0;
static unsigned int lo_sm = 0;
static unsigned int lo_offset = 0;

// Хранилище текущего установленного тона (по умолчанию сброшено)
static uint8_t current_active_tone = VFO_TONE_NONE;

static volatile uint32_t dds_accumulator = 0;
static volatile uint32_t dds_step = 0;          
static volatile uint32_t target_pio_int = 8;    
static volatile uint32_t target_pio_frac8 = 0;  
static uint32_t current_clk_sys_hz = 120000000;
static struct repeating_timer sdr_dither_timer; 

// Высокоскоростной обработчик прерывания таймера (50 кГц) — инъекция фазы в PIO
static bool __not_in_flash_func(vfo_dither_callback)(struct repeating_timer *t) {
    dds_accumulator += dds_step;
    uint32_t current_frac = target_pio_frac8;
    uint32_t current_int  = target_pio_int;

    if (dds_accumulator < dds_step) { 
        current_frac++;
        if (current_frac > 255) {
            current_frac = 0;
            current_int++;
        }
    }
    lo_pio->sm[lo_sm].clkdiv = (current_int << 16) | (current_frac << 8);
    return true; 
}

// Защитное отключение периферии от PLL_SYS, чтобы не падал USB/UART при фиксации клокинга
static void detach_peripheral_clock() {
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * 1000000, 48 * 1000000);
}

// Математический расчет аппаратных коэффициентов частоты
static VfoParameters calculate_freq_params(unsigned int target_frequency_hz) {
    if (target_frequency_hz < 100000)   target_frequency_hz = 100000;
    if (target_frequency_hz > 35000000) target_frequency_hz = 35000000;

    uint64_t crystal_hz = VFO_CALIBRATED_XOSC_HZ; 
    unsigned int p1 = 5, p2 = 2, fbdiv_int = 100; 
    uint64_t clk_sys_hz = ((uint64_t)fbdiv_int * crystal_hz) / (uint64_t)(p1 * p2);

    double clocks_per_period = 2.0; 
    uint64_t pio_denom = (uint64_t)target_frequency_hz * (uint64_t)clocks_per_period;
    uint64_t pio_div_fixed8 = (clk_sys_hz * 256ULL) / pio_denom;
    
    VfoParameters params;
    params.pio_int  = pio_div_fixed8 >> 8;
    params.pio_frac = pio_div_fixed8 & 0xFFu;

    if (params.pio_int < 2) { params.pio_int = 2; params.pio_frac = 0; }

    uint64_t clk_sys_rem = (clk_sys_hz * 256ULL) % pio_denom;
    params.dds_step = (uint32_t)((clk_sys_rem * 4294967296ULL) / pio_denom);
    
    return params;
}

// Реализация инициализации железа и заполнения таблицы тонов
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz) {
    // Сбрасываем внутренний трекер тона перед новой инициализацией
    current_active_tone = VFO_TONE_NONE;

    // 1. Стабилизация тактовой частоты процессора жестко на 120 МГц
    detach_peripheral_clock();
    uint32_t ints_status = save_and_disable_interrupts();
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12 * 1000000, 12 * 1000000);
    reset_block(RESETS_RESET_PLL_SYS_BITS);
    unreset_block_wait(RESETS_RESET_PLL_SYS_BITS);
    pll_init(pll_sys, 1, 1200000000ULL, 5, 2); 
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 120000000, 120000000);
    restore_interrupts(ints_status);

    unsigned int p1 = 5, p2 = 2, fbdiv_int = 100;
    current_clk_sys_hz = (uint32_t)(((uint64_t)fbdiv_int * VFO_CALIBRATED_XOSC_HZ) / (uint64_t)(p1 * p2));

    // 2. Настройка аппаратного блока PIO под VFO_OUTPUT_PIN
    lo_offset = pio_add_program(lo_pio, &pio_square_program);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, lo_offset + 0, lo_offset + 1);
    sm_config_set_set_pins(&c, VFO_OUTPUT_PIN, 1);
    
    pio_gpio_init(lo_pio, VFO_OUTPUT_PIN); 
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, VFO_OUTPUT_PIN, 1, true); 
    
    pio_sm_init(lo_pio, lo_sm, lo_offset, &c);
    pio_sm_set_enabled(lo_pio, lo_sm, true);

    // 3. Предрасчет всей сетки частот в ОЗУ
    for (int i = 0; i < VFO_IFKP_TONES_COUNT; i++) {
        unsigned int tone_freq = (unsigned int)(base_freq_hz + (i * step_hz));
        ifkp_tones[i] = calculate_freq_params(tone_freq);
    }

    // 4. Начальная установка на Тон 0 и запуск таймера дизеринга (интервал 20 мкс)
    vfo_set_tone_instant(0);
    add_repeating_timer_us(-20, vfo_dither_callback, NULL, &sdr_dither_timer);
}

// Реализация мгновенного переключения тона С ОПТИМИЗАЦИЕЙ ПОВТОРОВ
void __not_in_flash_func(vfo_set_tone_instant)(uint8_t tone_index) {
    if (tone_index >= VFO_IFKP_TONES_COUNT) return; 

    // === ВАЖНАЯ ОПТИМИЗАЦИЯ ===
    // Если новый тон совпадает с текущим активным — выходим сразу, ничего не переключая
    if (tone_index == current_active_tone) return;

    uint32_t ints_status = save_and_disable_interrupts();
    target_pio_int   = ifkp_tones[tone_index].pio_int;
    target_pio_frac8 = ifkp_tones[tone_index].pio_frac;
    dds_step         = ifkp_tones[tone_index].dds_step; 
    restore_interrupts(ints_status);

    // Запоминаем новый активный тон
    current_active_tone = tone_index;
}

// Реализация мгновенной телеграфной манипуляции
void __not_in_flash_func(vfo_set_cw_key)(bool key_down) {
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, VFO_OUTPUT_PIN, 1, key_down);
    
    // Если ключ "отжимается" (вывод выключается), сбрасываем состояние активного тона, 
    // чтобы при следующем замыкании ключа частота гарантированно принудительно обновилась.
    if (!key_down) {
        current_active_tone = VFO_TONE_NONE;
    }
}
