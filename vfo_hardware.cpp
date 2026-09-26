/**  
 * ============================================================================  
 *  vfo_hardware.cpp — Программный ВЧ-генератор (VFO) на PIO RP2040  
 *  Версия 2.10 (Профилирование и ASM-оптимизация Core 1), 2026-09-26  
 * ============================================================================  
 */

#include "vfo_hardware.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/resets.h"
#include "hardware/timer.h"
#include "hardware/structs/sio.h"
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
static const pio_program_t pio_square_program = { 
    .instructions = pio_square_instructions, 
    .length = 2, 
    .origin = -1,
    .pio_version = 0 
};

VfoParameters ifkp_tones[VFO_IFKP_TONES_COUNT];
static PIO lo_pio = pio0;
static unsigned int lo_sm = 0;

static unsigned int lo_offset = 0;
static bool pio_program_loaded = false;

#ifndef VFO_DITHER_ON_CORE1
static bool timer_already_running = false;
static struct repeating_timer sdr_dither_timer; 
#endif

static uint8_t current_active_tone = VFO_TONE_NONE;

// Межъядерный аппаратный спинлок
static spin_lock_t* vfo_spin_lock = nullptr;

// Переменные обмена данными между ядрами (Защищены спинлоком)
static volatile uint32_t dds_step = 0;          
static volatile uint32_t target_pio_int = 8;    
static volatile uint32_t target_pio_frac8 = 0;  
static volatile bool tone_changed = false; 

// Глобальное состояние DDS-накопителей в ОЗУ (используется в С-ориентированных ветках)
static volatile uint32_t dds_accumulator = 0; 
#ifdef VFO_USE_MASH2
static volatile uint32_t dds_accum_m2 = 0;    
static volatile uint32_t m2_carry_prev = 0;   
#endif

// Состояние встроенного ГПСЧ Xorshift32
#define VFO_RAND_SEED_INIT 0xACE1u
static volatile uint32_t xorshift_state = VFO_RAND_SEED_INIT;

static uint32_t current_clk_sys_hz = 120000000;

/**
 * Быстрый генератор псевдослучайных чисел Xorshift32 в ОЗУ.
 */
static inline uint32_t __not_in_flash_func(vfo_xorshift32_raw)(uint32_t state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

static inline uint32_t __not_in_flash_func(vfo_xorshift32)() {
    uint32_t x = xorshift_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xorshift_state = x;
    return x;
}

/**
 * Единичный атомарный шаг расчета дизеринга (Эталонная Си-версия).
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
#ifndef VFO_DITHER_ON_CORE1
static bool __not_in_flash_func(vfo_dither_callback)(struct repeating_timer *t) {
    (void)t; 
    vfo_dither_step(dds_step, target_pio_int, target_pio_frac8);
    return true; 
}
#endif

/**
 * Главная точка входа для второго ядра (Core 1).
 */
static void __not_in_flash_func(vfo_core1_entry)() {
    // Локальные копии параметров тона
    uint32_t l_step = 0;
    uint32_t l_int = 8;
    uint32_t l_frac = 0;

    // Локальные аккумуляторы состояния (DDS и ГПСЧ) для разрыва зависимостей по ОЗУ
    uint32_t loc_acc1 = 0;
    uint32_t loc_rand_state = VFO_RAND_SEED_INIT;
#ifdef VFO_USE_MASH2
    uint32_t loc_acc2 = 0;
    uint32_t loc_m2_carry_prev = 0;
#endif

    // Адрес целевого регистра clkdiv для прямой ASM-записи мимо структуры PIO
    volatile uint32_t *clkdiv_reg = &lo_pio->sm[lo_sm].clkdiv;
    
    // Битовая маска для аппаратного тоггла пина отладки
    const uint32_t profile_pin_mask = (1u << VFO_PROFILE_PIN);

    while (true) {
        // Проверка смены тона на Си-уровне
        if (__builtin_expect(tone_changed, 0)) {
            uint32_t save = spin_lock_blocking(vfo_spin_lock);
            
            l_step = dds_step;
            l_int  = target_pio_int;
            l_frac = target_pio_frac8;
            
            loc_acc1 = 0;
            loc_rand_state = xorshift_state;
#ifdef VFO_USE_MASH2
            loc_acc2 = 0;
            loc_m2_carry_prev = 0;
#endif
            tone_changed = false; 
            
            spin_unlock(vfo_spin_lock, save);
        }

#ifdef VFO_DITHER_PROFILE
        // Шаг 1: Аппаратный тоггл пина через SIO (выполняется за 1 такт)
        sio_hw->gpio_togl = profile_pin_mask;
#endif






#ifdef VFO_DITHER_FAST
        // === ВЕТКА УЛЬТРАЗВУКОВОЙ ASM-ОПТИМИЗАЦИИ (С дросселированием шины PIO) ===
        uint32_t step = l_step;

#ifdef VFO_DITHER_RANDOMIZE
        // Быстрый Xorshift в локальном регистре
        loc_rand_state = vfo_xorshift32_raw(loc_rand_state);
        int32_t r_bits = (int32_t)(loc_rand_state & ((1u << VFO_DITHER_RAND_BITS) - 1));
        int32_t r_dither = r_bits - (1 << (VFO_DITHER_RAND_BITS - 1));
        if (r_dither == -(1 << (VFO_DITHER_RAND_BITS - 1))) {
            r_dither = 0; 
        }
        step = (uint32_t)((int32_t)step + r_dither);
#endif

        // Структура контекста в ОЗУ для разгрузки регистров компилятора
        struct {
            uint32_t acc1;
            uint32_t acc2;
            uint32_t step;
            uint32_t m2_carry_prev;
            uint32_t pio_int;
            uint32_t pio_frac;
            volatile uint32_t *pio_clkdiv;
        } __attribute__((packed)) ctx;

        ctx.acc1 = loc_acc1;
        ctx.step = step;
        ctx.pio_int = l_int;
        ctx.pio_frac = l_frac;
        ctx.pio_clkdiv = clkdiv_reg;
#ifdef VFO_USE_MASH2
        ctx.acc2 = loc_acc2;
        ctx.m2_carry_prev = loc_m2_carry_prev;
#else
        ctx.acc2 = 0;
        ctx.m2_carry_prev = 0;
#endif

        asm volatile (
            ".thumb             \n\t"
            ".syntax unified    \n\t"
            
            // Загрузка данных из ОЗУ структуры в нижние регистры r0-r4
            "ldr  r0, [%0, #0]  \n\t"  // r0 = ctx.acc1
            "ldr  r2, [%0, #8]  \n\t"  // r2 = ctx.step
            "movs r1, #0        \n\t"  // r1 = carry1 = 0

#ifdef VFO_USE_MASH2
            "ldr  r3, [%0, #4]  \n\t"  // r3 = ctx.acc2
            "movs r4, #0        \n\t"  // r4 = carry2 = 0

            // Вычисление MASH-2
            "adds r0, r2        \n\t"  // r0 (acc1) += r2 (step)
            "adcs r1, r1        \n\t"  // r1 (carry1) = r1 + r1 + C
            "adds r3, r0        \n\t"  // r3 (acc2) += r0 (acc1)
            "adcs r4, r4        \n\t"  // r4 (carry2) = r4 + r4 + C

            "str  r3, [%0, #4]  \n\t"  // Сохраняем обновленный acc2 обратно в структуру
            
            // Расчет коррекции с защитой от затирания
            "adds r1, r4        \n\t"  // r1 = carry1 + carry2
            "ldr  r2, [%0, #12] \n\t"  // r2 = СТАРЫЙ ctx.m2_carry_prev (загружаем до перезаписи!)
            "subs r1, r2        \n\t"  // r1 = total_correction = (carry1 + carry2) - old_prev
            
            // Фиксация текущего переноса для следующего шага
            "str  r4, [%0, #12] \n\t"  // ctx.m2_carry_prev = r4 (текущий carry2)
#else


            // Вычисление MASH-1
            "adds r0, r2        \n\t"  // r0 (acc1) += r2 (step)
            "adcs r1, r1        \n\t"  // r1 (total_correction) = r1 + r1 + C
#endif
            "str  r0, [%0, #0]  \n\t"  // Сохраняем обновленный acc1 обратно

            // Вычисление нового FRAC и знаковая коррекция INT
            "ldr  r2, [%0, #20] \n\t"  // r2 = ctx.pio_frac
            "adds r2, r1        \n\t"  // r2 (current_frac) = pio_frac + total_correction
            "movs r3, r2        \n\t"  // r3 = копия current_frac перед наложением маски
            
            "asrs r2, r2, #8    \n\t"  // r2 = знаковый сдвиг (current_frac >> 8)
            "ldr  r1, [%0, #16] \n\t"  // r1 = ctx.pio_int
            "adds r1, r2        \n\t"  // r1 (current_int) = pio_int + коррекция
            
            "movs r4, #255      \n\t"  // r4 = 0xFF
            "ands r3, r4        \n\t"  // r3 (current_frac) &= r4 (0xFF)

            // Сборка 32-битного clkdiv 
            "lsls r1, r1, #16   \n\t"  // r1 = current_int << 16
            "lsls r3, r3, #8    \n\t"  // r3 = current_frac << 8
            "orrs r1, r3        \n\t"  // r1 = clkdiv = r1 | r3
            
            // Прямая запись в шину PIO
            "ldr  r0, [%0, #24] \n\t"  // r0 = ctx.pio_clkdiv
            "str  r1, [r0]      \n\t"  // *pio_clkdiv = clkdiv

            // === ДОБАВЛЕНО: ДОЗИРОВАННОЕ ДРОССЕЛИРОВАНИЕ ЦИКЛА ===
            // 8 тактов NOP создают безопасное терапевтическое окно для автомата PIO.
            // Частота записи упадет до ~8-10 МГц, чего более чем достаточно для Noise Shaping,
            // но хаотичный шум («белая стена») полностью исчезнет.
            "nop \n\t" "nop \n\t" "nop \n\t" "nop \n\t"
            "nop \n\t" "nop \n\t" "nop \n\t" "nop \n\t"
            :
            : "r" (&ctx)
            : "r0", "r1", "r2", "r3", "r4", "memory", "cc"
        );

        // Возвращаем результаты обратно в Си-переменные рантайма
        loc_acc1 = ctx.acc1;
#ifdef VFO_USE_MASH2
        loc_acc2 = ctx.acc2;
        loc_m2_carry_prev = ctx.m2_carry_prev;
#endif

#else










        // === ЭТАЛОННАЯ СИ-ВЕРСИЯ (Без оптимизации, для сравнения) ===
        // Принудительно гоним данные в глобальные volatile, чтобы работала базовая Си-функция
        dds_accumulator = loc_acc1;
        xorshift_state = loc_rand_state;
#ifdef VFO_USE_MASH2
        dds_accum_m2 = loc_acc2;
        m2_carry_prev = loc_m2_carry_prev;
#endif

        vfo_dither_step(l_step, l_int, l_frac);

        loc_acc1 = dds_accumulator;
        loc_rand_state = xorshift_state;
#ifdef VFO_USE_MASH2
        loc_acc2 = dds_accum_m2;
        loc_m2_carry_prev = m2_carry_prev;
#endif
#endif
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
    uint64_t min_full_frac_metric = 0xFFFFFFFFFFULL; 
    VfoParameters best_params = calculate_raw_params_chz(clk_sys_hz, base_target_chz);

    for (int32_t offset_chz = -10; offset_chz <= 10; offset_chz++) {
        uint64_t candidate_chz = (uint64_t)((int64_t)base_target_chz + offset_chz);
        VfoParameters candidate_params = calculate_raw_params_chz(clk_sys_hz, candidate_chz);
        
        uint64_t full_frac = ((uint64_t)candidate_params.pio_frac << 32) | candidate_params.dds_step;
        
        uint64_t dist_to_0 = full_frac;
        uint64_t dist_to_max = (1ULL << 40) - full_frac;
        uint64_t current_metric = (dist_to_0 < dist_to_max) ? dist_to_0 : dist_to_max;

        if (current_metric < min_full_frac_metric) {
            min_full_frac_metric = current_metric;
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
 */
static PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz) {
    uint64_t crystal_hz = VFO_CALIBRATED_XOSC_HZ;
    uint64_t base_target_chz = (uint64_t)target_frequency_hz * 100ULL;
    
    PllConfig best_pll = { 100, 5, 2, 120000000ULL }; 
#ifdef VFO_CLOCK_133_MHZ
    best_pll = { 133, 6, 2, 133003879ULL };
#endif

    uint64_t min_full_frac_metric = 0xFFFFFFFFFFULL; 

    for (uint32_t p1 = 2; p1 <= 6; p1++) {
        for (uint32_t p2 = 1; p2 <= 2; p2++) {
            uint32_t pdiv_total = p1 * p2;
            
            for (uint32_t fbdiv = 30; fbdiv <= 150; fbdiv++) {
                uint64_t vco_hz = fbdiv * crystal_hz;
                
                if (vco_hz < 750000000ULL || vco_hz > 1600000000ULL) continue;
                
                uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;
                
                if (clk_sys_hz < 100000000ULL || clk_sys_hz > 133000000ULL) continue;
                
                uint64_t clocks_per_period = 2ULL;
                uint64_t pio_denom = base_target_chz * clocks_per_period;
                uint64_t pio_div_fixed8 = ((clk_sys_hz * 256ULL) * 100ULL) / pio_denom;
                
                uint32_t test_pio_int = pio_div_fixed8 >> 8;
                if (test_pio_int < 2) continue; 
                
                uint32_t test_pio_frac = pio_div_fixed8 & 0xFFu;
                
                uint64_t clk_sys_rem = ((clk_sys_hz * 256ULL) * 100ULL) % pio_denom;
                uint64_t intermediate = (clk_sys_rem << 16) / pio_denom;
                uint64_t remainder_low = (clk_sys_rem << 16) % pio_denom;
                uint32_t test_dds_step = (uint32_t)((intermediate << 16) + ((remainder_low << 16) / pio_denom));
                
                uint64_t full_frac = ((uint64_t)test_pio_frac << 32) | test_dds_step;
                
                uint64_t dist_to_0 = full_frac;
                uint64_t dist_to_max = (1ULL << 40) - full_frac;
                uint64_t current_metric = (dist_to_0 < dist_to_max) ? dist_to_0 : dist_to_max;

                if (current_metric < min_full_frac_metric || 
                   (current_metric == min_full_frac_metric && clk_sys_hz > best_pll.clk_sys_hz)) {
                    
                    min_full_frac_metric = current_metric;
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

// === ИНИЦИАЛИЗАЦИЯ И СТАРТ СИСТЕМЫ ===
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz) {
#ifdef VFO_DITHER_PROFILE
    // Инициализация отладочного пина под замер частоты цикла дизеринга
    gpio_init(VFO_PROFILE_PIN);
    gpio_set_dir(VFO_PROFILE_PIN, GPIO_OUT);
#endif

#ifdef VFO_DITHER_ON_CORE1
    multicore_reset_core1(); 
#else
    if (timer_already_running) {
        cancel_repeating_timer(&sdr_dither_timer);
        timer_already_running = false;
    }
#endif

    current_active_tone = VFO_TONE_NONE;
    xorshift_state = VFO_RAND_SEED_INIT + time_us_32(); 

    if (vfo_spin_lock == nullptr) {
        int lock_id = spin_lock_claim_unused(true);
        vfo_spin_lock = spin_lock_init(lock_id);
    }

    uint32_t best_fbdiv = 100, best_p1 = 5, best_p2 = 2;
    uint64_t clk_sys_target_hz = 120000000ULL;

#ifdef VFO_PLL_AUTOTUNE
    PllConfig optimal_pll = vfo_find_optimal_pll(base_freq_hz);
    best_fbdiv = optimal_pll.fbdiv;
    best_p1 = optimal_pll.p1;
    best_p2 = optimal_pll.p2;
    clk_sys_target_hz = optimal_pll.clk_sys_hz;
#else
#ifdef VFO_CLOCK_133_MHZ
    best_fbdiv = 133; best_p1 = 6; best_p2 = 2; clk_sys_target_hz = 133000000ULL;
#endif
#endif

    detach_peripheral_clock(); 
    uint32_t ints_status = save_and_disable_interrupts();
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12 * 1000000, 12 * 1000000);
    reset_block(RESETS_RESET_PLL_SYS_BITS);
    unreset_block_wait(RESETS_RESET_PLL_SYS_BITS);

    uint32_t vco_nominal_hz = (uint32_t)(VFO_CALIBRATED_XOSC_HZ * (uint64_t)best_fbdiv);
    pll_init(pll_sys, 1, vco_nominal_hz, best_p1, best_p2); 
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, (uint32_t)clk_sys_target_hz, (uint32_t)clk_sys_target_hz);
    restore_interrupts(ints_status);

    current_clk_sys_hz = (uint32_t)(((uint64_t)best_fbdiv * VFO_CALIBRATED_XOSC_HZ) / (uint64_t)(best_p1 * best_p2));

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

    for (int i = 0; i < VFO_IFKP_TONES_COUNT; i++) {
        unsigned int tone_freq = (unsigned int)(base_freq_hz + (i * step_hz));
        ifkp_tones[i] = calculate_freq_params(current_clk_sys_hz, tone_freq);
    }

    VfoParameters base_params = calculate_freq_params(current_clk_sys_hz, base_freq_hz);
    Serial.printf("\n--- VFO Core 1 ASM Optimization Active ---\n");
    Serial.printf("Target Freq: %u Hz\n", base_freq_hz);
    Serial.printf("clk_sys    : %u Hz\n", current_clk_sys_hz);
    Serial.printf("PIO Regs   : INT=%u, FRAC=%u\n", base_params.pio_int, base_params.pio_frac);
    Serial.printf("DDS Step   : 0x%08X\n", base_params.dds_step);
#ifdef VFO_DITHER_FAST
    Serial.printf("Dither Mode: VFO_DITHER_FAST (ASM Cortex-M0+)\n");
#else
    Serial.printf("Dither Mode: Standard C-Version\n");
#endif
    Serial.printf("-----------------------------------------\n");

    vfo_set_tone_instant(0);

#ifdef VFO_DITHER_ON_CORE1
    tone_changed = true;
    multicore_launch_core1(vfo_core1_entry); 
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
