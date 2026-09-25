#include "vfo_hardware2.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/resets.h"
#include "hardware/timer.h"
#include "pico/multicore.h"

// Структура для возврата найденных физических коэффициентов PLL
struct PllConfig {
    uint32_t fbdiv;
    uint32_t p1;
    uint32_t p2;
    uint64_t clk_sys_hz;
};

// Ассемблерная микропрограмма PIO для меандра (цикл из 2 тактов)
static const uint16_t pio_square_instructions[] = { 0xe001, 0xe000 };
static const pio_program_t pio_square_program = { .instructions = pio_square_instructions, .length = 2, .origin = -1 };

VfoParameters ifkp_tones[VFO_IFKP_TONES_COUNT];
static PIO lo_pio = pio0;
static unsigned int lo_sm = 0;

static unsigned int lo_offset = 0;
static bool pio_program_loaded = false;
static bool timer_already_running = false;

static uint8_t current_active_tone = VFO_TONE_NONE;

// Межъядерный аппаратный спинлок
static spin_lock_t* vfo_spin_lock = nullptr;

// Переменные обмена данными между ядрами (Защищены спинлоком)
static volatile uint32_t dds_step = 0;          
static volatile uint32_t target_pio_int = 8;    
static volatile uint32_t target_pio_frac8 = 0;  
static volatile bool tone_changed = false; 

// Глобальное состояние DDS-накопителей в ОЗУ
static volatile uint32_t dds_accumulator = 0; 
#ifdef VFO_USE_MASH2
static volatile uint32_t dds_accum_m2 = 0;    
static volatile uint32_t m2_carry_prev = 0;   
#endif

// Состояние встроенного ГПСЧ Xorshift32
#define VFO_RAND_SEED_INIT 0xACE1u
static volatile uint32_t xorshift_state = VFO_RAND_SEED_INIT;

static uint32_t current_clk_sys_hz = 120000000;
static struct repeating_timer sdr_dither_timer; 

/**
 * Быстрый генератор псевдослучайных чисел Xorshift32 в ОЗУ.
 */
static inline uint32_t __not_in_flash_func(vfo_xorshift32)() {
    uint32_t x = xorshift_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xorshift_state = x;
    return x;
}

/**
 * Единичный атомарный шаг расчета дизеринга (MASH-1-1 + Рандомизация).
 */
static inline void __not_in_flash_func(vfo_dither_step)(uint32_t local_step, uint32_t local_int, uint32_t local_frac) {
    uint32_t step = local_step;

#ifdef VFO_DITHER_RANDOMIZE
    int32_t r_bits = (int32_t)(vfo_xorshift32() & ((1u << VFO_DITHER_RAND_BITS) - 1));
    int32_t r_dither = r_bits - (1 << (VFO_DITHER_RAND_BITS - 1));
    if (r_dither == -(1 << (VFO_DITHER_RAND_BITS - 1))) {
        r_dither = 0; 
    }
    step = (uint32_t)((int32_t)step + r_dither);
#endif

    int32_t total_correction = 0;

#ifdef VFO_USE_MASH2
    uint32_t old_acc1 = dds_accumulator;
    dds_accumulator += step;
    uint32_t carry1 = (dds_accumulator < old_acc1) ? 1 : 0; 

    uint32_t old_acc2 = dds_accum_m2;
    dds_accum_m2 += dds_accumulator;
    uint32_t carry2 = (dds_accum_m2 < old_acc2) ? 1 : 0; 

    total_correction = (int32_t)carry1 + (int32_t)carry2 - (int32_t)m2_carry_prev;
    m2_carry_prev = carry2; 
#else
    uint32_t old_acc = dds_accumulator;
    dds_accumulator += step;
    if (dds_accumulator < old_acc) {
        total_correction = 1;
    }
#endif

    int32_t current_frac = (int32_t)local_frac + total_correction;
    int32_t current_int  = (int32_t)local_int;

    while (current_frac > 255) {
        current_frac -= 256;
        current_int++;
    }
    while (current_frac < 0) {
        current_frac += 256;
        current_int--;
    }

    lo_pio->sm[lo_sm].clkdiv = ((uint32_t)current_int << 16) | ((uint32_t)current_frac << 8);
}

/**
 * Таймерный колбэк (Для режима Core 0).
 */
static bool __not_in_flash_func(vfo_dither_callback)(struct repeating_timer *t) {
    vfo_dither_step(dds_step, target_pio_int, target_pio_frac8);
    return true; 
}

/**
 * Главная точка входа для второго ядра (Core 1).
 */
static void __not_in_flash_func(vfo_core1_entry)() {
    uint32_t l_step = 0;
    uint32_t l_int = 8;
    uint32_t l_frac = 0;

    while (true) {
        if (tone_changed) {
            uint32_t save = spin_lock_blocking(vfo_spin_lock);
            
            l_step = dds_step;
            l_int  = target_pio_int;
            l_frac = target_pio_frac8;
            
#ifdef VFO_USE_MASH2
            dds_accum_m2 = 0;
            m2_carry_prev = 0;
#endif
            dds_accumulator = 0;
            tone_changed = false; 
            
            spin_unlock(vfo_spin_lock, save);
        }

        vfo_dither_step(l_step, l_int, l_frac);
    }
}

static void detach_peripheral_clock() {
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * 1000000, 48 * 1000000);
}

/**
 * Прецизионный целочисленный расчет параметров в сантигерцах.
 */
static VfoParameters calculate_raw_params_chz(uint64_t clk_sys_hz, uint64_t chz_target) {
    if (chz_target < 10000000ULL)   chz_target = 10000000ULL;
    if (chz_target > 3000000000ULL) chz_target = 3000000000ULL;

    uint64_t clocks_per_period = 2ULL; 
    uint64_t pio_denom = chz_target * clocks_per_period; 
    
    uint64_t pio_div_fixed8 = ((clk_sys_hz * 256ULL) * 100ULL) / pio_denom;
    
    VfoParameters params;
    params.pio_int  = pio_div_fixed8 >> 8;
    params.pio_frac = pio_div_fixed8 & 0xFFu;
    params.target_freq_chz = (uint32_t)chz_target;

    if (params.pio_int < 2) { params.pio_int = 2; params.pio_frac = 0; }

    uint64_t clk_sys_rem = ((clk_sys_hz * 256ULL) * 100ULL) % pio_denom;
    
    uint64_t intermediate = (clk_sys_rem << 16) / pio_denom;
    uint64_t remainder_low = (clk_sys_rem << 16) % pio_denom;
    
    params.dds_step = (uint32_t)((intermediate << 16) + ((remainder_low << 16) / pio_denom));
    
    return params;
}

/**
 * Расчет аппаратных коэффициентов частоты с Grid Snapping от внешней clk_sys_hz.
 */
static VfoParameters calculate_freq_params(uint64_t clk_sys_hz, unsigned int target_frequency_hz) {
    uint64_t base_target_chz = (uint64_t)target_frequency_hz * 100ULL;

#ifdef VFO_SNAP_TO_GRID
    uint32_t min_dither_metric = 0xFFFFFFFFu;
    VfoParameters best_params = calculate_raw_params_chz(clk_sys_hz, base_target_chz);

    for (int32_t offset_chz = -10; offset_chz <= 10; offset_chz++) {
        uint64_t candidate_chz = (uint64_t)((int64_t)base_target_chz + offset_chz);
        VfoParameters candidate_params = calculate_raw_params_chz(clk_sys_hz, candidate_chz);
        
        uint32_t dist_to_0 = candidate_params.dds_step;
        uint32_t dist_to_max = 0xFFFFFFFFu - candidate_params.dds_step;
        uint32_t current_metric = (dist_to_0 < dist_to_max) ? dist_to_0 : dist_to_max;

        if (current_metric < min_dither_metric) {
            min_dither_metric = current_metric;
            best_params = candidate_params;
        }
    }
    return best_params;
#else
    return calculate_raw_params_chz(clk_sys_hz, base_target_chz);
#endif
}

/**
 * Сканирующий матричный алгоритм поиска оптимальной частоты PLL (clk_sys).
 * Минимизирует остаток dds_step для целевой частоты.
 */
static PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz) {
    uint64_t crystal_hz = VFO_CALIBRATED_XOSC_HZ;
    uint64_t base_target_chz = (uint64_t)target_frequency_hz * 100ULL;
    
    PllConfig best_pll = { 100, 5, 2, 120000000ULL }; 
#ifdef VFO_CLOCK_133_MHZ
    best_pll = { 133, 6, 2, 133003879ULL };
#endif

    uint32_t min_dither_metric = 0xFFFFFFFFu;

    for (uint32_t p1 = 2; p1 <= 6; p1++) {
        for (uint32_t p2 = 1; p2 <= 2; p2++) {
            uint32_t pdiv_total = p1 * p2;
            
            for (uint32_t fbdiv = 30; fbdiv <= 150; fbdiv++) {
                uint64_t vco_hz = fbdiv * crystal_hz;
                
                if (vco_hz < 400000000ULL || vco_hz > 1200000000ULL) continue;
                
                uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;
                
                if (clk_sys_hz < 100000000ULL || clk_sys_hz > 133000000ULL) continue;
                
                uint64_t clocks_per_period = 2ULL;
                uint64_t pio_denom = base_target_chz * clocks_per_period;
                uint64_t pio_div_fixed8 = ((clk_sys_hz * 256ULL) * 100ULL) / pio_denom;
                
                if ((pio_div_fixed8 >> 8) < 2) continue; 
                
                uint64_t clk_sys_rem = ((clk_sys_hz * 256ULL) * 100ULL) % pio_denom;
                uint64_t intermediate = (clk_sys_rem << 16) / pio_denom;
                uint64_t remainder_low = (clk_sys_rem << 16) % pio_denom;
                uint32_t test_dds_step = (uint32_t)((intermediate << 16) + ((remainder_low << 16) / pio_denom));
                
                uint32_t dist_to_0 = test_dds_step;
                uint32_t dist_to_max = 0xFFFFFFFFu - test_dds_step;
                uint32_t current_metric = (dist_to_0 < dist_to_max) ? dist_to_0 : dist_to_max;
                
                if (current_metric < min_dither_metric) {
                    min_dither_metric = current_metric;
                    best_pll.fbdiv = fbdiv;
                    best_pll.p1 = p1;
                    best_pll.p2 = p2;
                    best_pll.clk_sys_hz = clk_sys_hz;
                }
            }
        }
    }
    return best_pll;
}

// === ПОЛНОСТЬЮ ВОССТАНОВЛЕННАЯ ФУНКЦИЯ ИНИЦИАЛИЗАЦИИ ===
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz) {
    // === 1. ЗАЩИТА СМЕНЫ СЕАНСА: Остановка старого dither-механизма ===
#ifdef VFO_DITHER_ON_CORE1
    multicore_reset_core1(); // Принудительно глушим старый плотный цикл Core 1
#else
    if (timer_already_running) {
        cancel_repeating_timer(&sdr_dither_timer);
        timer_already_running = false;
    }
#endif

    current_active_tone = VFO_TONE_NONE;
    xorshift_state = VFO_RAND_SEED_INIT + time_us_32(); 

    // Защищенное выделение аппаратного спинлока (строго один раз за аптайм)
    if (vfo_spin_lock == nullptr) {
        int lock_id = spin_lock_claim_unused(true);
        vfo_spin_lock = spin_lock_init(lock_id);
    }

    // Подготовка целевых коэффициентов тактирования PLL
    uint32_t best_fbdiv = 100, best_p1 = 5, best_p2 = 2;
    uint64_t clk_sys_target_hz = 120000000ULL;

#ifdef VFO_PLL_AUTOTUNE
    // Динамический автотюнинг PLL под базовую частоту текущей передачи
    PllConfig optimal_pll = vfo_find_optimal_pll(base_freq_hz);
    best_fbdiv = optimal_pll.fbdiv;
    best_p1 = optimal_pll.p1;
    best_p2 = optimal_pll.p2;
    clk_sys_target_hz = optimal_pll.clk_sys_hz;
#else
    // Статический режим на базе дефайнов
#ifdef VFO_CLOCK_133_MHZ
    best_fbdiv = 133; best_p1 = 6; best_p2 = 2; clk_sys_target_hz = 133000000ULL;
#endif
#endif

    // 2. Аппаратная перестройка системной тактовой частоты (clk_sys)
    detach_peripheral_clock(); // Спасаем USB/UART/Таймеры, уводя clk_peri на фиксированные 48 МГц
    uint32_t ints_status = save_and_disable_interrupts();
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12 * 1000000, 12 * 1000000);
    reset_block(RESETS_RESET_PLL_SYS_BITS);
    unreset_block_wait(RESETS_RESET_PLL_SYS_BITS);

    uint32_t vco_nominal_hz = (uint32_t)(VFO_CALIBRATED_XOSC_HZ * (uint64_t)best_fbdiv);
    pll_init(pll_sys, 1, vco_nominal_hz, best_p1, best_p2); 
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, (uint32_t)clk_sys_target_hz, (uint32_t)clk_sys_target_hz);
    restore_interrupts(ints_status);

    // Фиксация точной системной частоты для расчёта делителей
    current_clk_sys_hz = (uint32_t)(((uint64_t)best_fbdiv * VFO_CALIBRATED_XOSC_HZ) / (uint64_t)(best_p1 * best_p2));

    // === 3. ЗАЩИТА ПАМЯТИ PIO: Загрузка программы строго один раз ===
    if (!pio_program_loaded) {
        lo_offset = pio_add_program(lo_pio, &pio_square_program);
        pio_program_loaded = true;
    }
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, lo_offset + 0, lo_offset + 1);
    sm_config_set_set_pins(&c, VFO_OUTPUT_PIN, 1);
    
    pio_gpio_init(lo_pio, VFO_OUTPUT_PIN); 
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, VFO_OUTPUT_PIN, 1, true); 
    
    pio_sm_init(lo_pio, lo_sm, lo_offset, &c);
    pio_sm_set_enabled(lo_pio, lo_sm, true);

    // 4. Предрасчет всей сетки частот в ОЗУ от ФАКТИЧЕСКОЙ частоты current_clk_sys_hz
    for (int i = 0; i < VFO_IFKP_TONES_COUNT; i++) {
        unsigned int tone_freq = (unsigned int)(base_freq_hz + (i * step_hz));
        ifkp_tones[i] = calculate_freq_params(current_clk_sys_hz, tone_freq);
    }

    // 5. Безопасный чистый пуск dither-механизмов
    vfo_set_tone_instant(0);

#ifdef VFO_DITHER_ON_CORE1
    tone_changed = true;
    multicore_launch_core1(vfo_core1_entry); // Запускаем заново остановленное ядро
#else
    add_repeating_timer_us(-(int64_t)VFO_DITHER_INTERVAL_US, vfo_dither_callback, NULL, &sdr_dither_timer);
    timer_already_running = true;
#endif
}

void __not_in_flash_func(vfo_set_tone_instant)(uint8_t tone_index) {
    if (tone_index >= VFO_IFKP_TONES_COUNT) return; 
    if (tone_index == current_active_tone) return;

#ifdef VFO_DITHER_ON_CORE1
    uint32_t save = spin_lock_blocking(vfo_spin_lock);
    target_pio_int   = ifkp_tones[tone_index].pio_int;
    target_pio_frac8 = ifkp_tones[tone_index].pio_frac;
    dds_step         = ifkp_tones[tone_index].dds_step; 
    tone_changed     = true; 
    spin_unlock(vfo_spin_lock, save);
#else
    uint32_t ints_status = save_and_disable_interrupts();
    target_pio_int   = ifkp_tones[tone_index].pio_int;
    target_pio_frac8 = ifkp_tones[tone_index].pio_frac;
    dds_step         = ifkp_tones[tone_index].dds_step; 
#ifdef VFO_USE_MASH2
    dds_accum_m2 = 0;
    m2_carry_prev = 0;
#endif
    dds_accumulator = 0;
    restore_interrupts(ints_status);
#endif

    current_active_tone = tone_index;
}

void __not_in_flash_func(vfo_set_cw_key)(bool key_down) {
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, VFO_OUTPUT_PIN, 1, key_down);
    if (!key_down) {
        current_active_tone = VFO_TONE_NONE;
    }
}
