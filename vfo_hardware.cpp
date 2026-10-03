/**  
 * ============================================================================  
 *  vfo_hardware.cpp — Реализация гибридного PWM/MASH-2 двигателя КВ VFO
 * ============================================================================ 
 */

#include "vfo_hardware.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/resets.h"
#include "pico/multicore.h"

struct PllConfig {
    uint32_t fbdiv;
    uint32_t p1;
    uint32_t p2;
    uint64_t clk_sys_hz;
};

// Физическое размещение таблицы частот тонов в RAM
VfoParameters __attribute__((section(".time_critical.ifkp_tones"))) ifkp_tones[VFO_IFKP_TONES_COUNT];

extern bool dev_TX_state; 

static uint uint_slice_num = 0;
static uint uint_pwm_chan = 0;
static uint8_t current_active_tone = VFO_TONE_NONE;
static spin_lock_t* vfo_spin_lock = nullptr;

// Переменные межъядерного обмена (Защищены спинлоком)
static volatile uint32_t dds_step = 0;          
static volatile uint32_t target_pwm_wrap = 2;
static volatile uint32_t target_pwm_base_div = 16;  
static volatile bool tone_changed = false; 

// Резервное состояние для медленной Си-ветки
static volatile uint32_t dds_accumulator = 0; 
#ifdef VFO_USE_MASH2
static volatile uint32_t dds_accum_m2 = 0;    
static volatile uint32_t m2_carry_prev = 0;   
#endif

#define VFO_RAND_SEED_INIT 0xACE1u
static volatile uint32_t xorshift_state = VFO_RAND_SEED_INIT;
static uint32_t current_clk_sys_hz = 133000000;

/**
 * ПЕРЕВЕРНУТАЯ ПАРАМЕТРИЗАЦИЯ ШИМ ПОД КВ (Миллигерцы)
 * Фиксирует минимальный WRAP (2..4), уводя вес младшего бита FRAC в минимум.
 */
static VfoParameters calculate_raw_params_mhz(uint64_t clk_sys_hz, uint64_t mhz_target) {
    if (mhz_target < 100000000ULL)   mhz_target = 100000000ULL;
    if (mhz_target > 40000000000ULL) mhz_target = 40000000000ULL;

    VfoParameters params;
    params.target_freq_mhz = (uint32_t)mhz_target;

    uint32_t wrap = 2; // Базовое жесткое КВ-окно для меандра
    
    // На частотах ниже 10 МГц плавно увеличиваем wrap, чтобы разгрузить div_int (макс 255)
    if (mhz_target < 5000000000ULL)  wrap = 4;
    if (mhz_target < 2000000000ULL)  wrap = 8;

    params.pwm_wrap = wrap;

    // Рассчитываем крупный clkdiv (формат 8.4) под выбранный маленький wrap
    uint64_t clkdiv_fx4 = (clk_sys_hz * 16ULL * 1000ULL) / (2ULL * (uint64_t)wrap * mhz_target);

    // Жесткие лимиты аппаратного регистра CH_DIV (формат 8.4)
    if (clkdiv_fx4 > 4095) clkdiv_fx4 = 4095; // Максимум 255.15
    if (clkdiv_fx4 < 16)   clkdiv_fx4 = 16;   // Минимум 1.0

    params.pwm_base_div_fx4 = (uint32_t)clkdiv_fx4;

    // Расчет 32-битного остатка ошибки относительно истинной текущей раскладки ШИМ
    uint64_t actual_div_scaled = clkdiv_fx4 * (uint64_t)wrap;
    uint64_t vfo_denom = mhz_target * 2ULL * actual_div_scaled;
    uint64_t clk_sys_rem = ((clk_sys_hz * 16ULL * 1000ULL) % vfo_denom);
    
    uint64_t intermediate = (clk_sys_rem << 16) / vfo_denom;
    uint64_t remainder_low = (clk_sys_rem << 16) % vfo_denom;
    params.dds_step = (uint32_t)((intermediate << 16) + ((remainder_low << 16) / vfo_denom));

    return params;
}

/**
 * Оптимизация автотюна PLL под КВ-раскладку параметров
 */
static PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz) {
    uint64_t crystal_hz = VFO_CALIBRATED_XOSC_HZ;
    uint64_t base_target_mhz = (uint64_t)target_frequency_hz * 1000ULL;
    
    PllConfig best_pll = { 133, 6, 2, 133000000ULL };
    uint32_t min_dds_metric = 0xFFFFFFFFu;

    for (uint32_t p1 = 2; p1 <= 6; p1++) {
        for (uint32_t p2 = 1; p2 <= 2; p2++) {
            uint32_t pdiv_total = p1 * p2;
            for (uint32_t fbdiv = 30; fbdiv <= 150; fbdiv++) {
                uint64_t vco_hz = fbdiv * crystal_hz;
                if (vco_hz < 750000000ULL || vco_hz > 1600000000ULL) continue;
                
                uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;
                if (clk_sys_hz < 100000000ULL || clk_sys_hz > 133400000ULL) continue;

                VfoParameters test_params = calculate_raw_params_mhz(clk_sys_hz, base_target_mhz);
                uint32_t dstep = test_params.dds_step;
                uint32_t dist_to_0 = dstep;
                uint32_t dist_to_max = 0xFFFFFFFFu - dstep;
                uint32_t current_metric = (dist_to_0 < dist_to_max) ? dist_to_0 : dist_to_max;

                if (current_metric < min_dds_metric) {
                    min_dds_metric = current_metric;
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

/**
 * Единичный шаг дизеринга (Эталонная Си-версия через ОЗУ)
 * Убрана мертвая переменная local_wrap, добавлены двусторонние лимиты клемпа (16..4095)
 */
static inline void __not_in_flash_func(vfo_dither_step)(uint32_t local_step, uint32_t local_div_fx4) {
    uint32_t step = local_step;

#ifdef VFO_DITHER_RANDOMIZE
    uint32_t x = xorshift_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    xorshift_state = x;
    int32_t r_bits = (int32_t)(x & ((1u << VFO_DITHER_RAND_BITS) - 1));
    int32_t r_dither = r_bits - (1 << (VFO_DITHER_RAND_BITS - 1));
    if (r_dither == -(1 << (VFO_DITHER_RAND_BITS - 1))) r_dither = 0;
    step = (uint32_t)((int32_t)step + r_dither);
#endif

    int32_t total_correction = 0;
#ifdef VFO_USE_MASH2
    uint32_t old_acc1 = dds_accumulator; dds_accumulator += step;
    uint32_t carry1 = (dds_accumulator < old_acc1) ? 1 : 0; 
    uint32_t old_acc2 = dds_accum_m2; dds_accum_m2 += dds_accumulator;
    uint32_t carry2 = (dds_accum_m2 < old_acc2) ? 1 : 0; 
    total_correction = (int32_t)carry1 + (int32_t)carry2 - (int32_t)m2_carry_prev;
    m2_carry_prev = carry2; 
#else
    uint32_t old_acc = dds_accumulator; dds_accumulator += step;
    if (dds_accumulator < old_acc) total_correction = 1;
#endif

    int32_t final_div_fx4 = (int32_t)local_div_fx4 + total_correction;
    
    // Двусторонний аппаратный клемп 12-битного регистра CH_DIV (1.0 .. 255.15)
    if (final_div_fx4 < 16)   final_div_fx4 = 16; 
    if (final_div_fx4 > 4095) final_div_fx4 = 4095;

    pwm_hw->slice[uint_slice_num].div = (uint32_t)final_div_fx4;
}

/**
 * Точка входа Движка Дизеринга на Core 1
 */
static void __not_in_flash_func(vfo_core1_entry)() {
    int32_t l_step = 0;
    int32_t l_div_fx4 = 16;

    uint32_t loc_acc1 = 0;
    uint32_t loc_rand_state = VFO_RAND_SEED_INIT;
#ifdef VFO_USE_MASH2
    uint32_t loc_acc2 = 0;
    uint32_t loc_m2_carry_prev = 0;
#endif

    volatile uint32_t *pwm_div_reg = &pwm_hw->slice[uint_slice_num].div;

    while (true) {
        if (__builtin_expect(tone_changed, 0)) {
            uint32_t save = spin_lock_blocking(vfo_spin_lock);
            l_step    = (int32_t)dds_step;
            l_div_fx4 = (int32_t)target_pwm_base_div;
            
            loc_acc1 = 0;
            loc_rand_state = xorshift_state;
#ifdef VFO_USE_MASH2
            loc_acc2 = 0;
            loc_m2_carry_prev = 0;
#endif
            tone_changed = false; 
            spin_unlock(vfo_spin_lock, save);
        }

#ifdef VFO_DITHER_FAST
        // === ВЫСОКОСКОРОСТНОЙ РЕГИСТРОВЫЙ КОНВЕЙЕР MASH-2 ===
        int32_t step = l_step;

#ifdef VFO_DITHER_RANDOMIZE
        loc_rand_state ^= loc_rand_state << 13;
        loc_rand_state ^= loc_rand_state >> 17;
        loc_rand_state ^= loc_rand_state << 5;
        
        int32_t r_bits = (int32_t)(loc_rand_state & ((1u << VFO_DITHER_RAND_BITS) - 1));
        int32_t r_dither = r_bits - (1 << (VFO_DITHER_RAND_BITS - 1));
        if (__builtin_expect(r_dither == -(1 << (VFO_DITHER_RAND_BITS - 1)), 0)) r_dither = 0; 
        step = (int32_t)((int32_t)step + r_dither);
#endif

        int32_t total_correction = 0;
#ifdef VFO_USE_MASH2
        uint32_t old_acc1 = loc_acc1; loc_acc1 += (uint32_t)step;
        uint32_t carry1 = (loc_acc1 < old_acc1) ? 1 : 0; 

        uint32_t old_acc2 = loc_acc2; loc_acc2 += loc_acc1;
        uint32_t carry2 = (loc_acc2 < old_acc2) ? 1 : 0; 

        total_correction = (int32_t)carry1 + (int32_t)carry2 - (int32_t)loc_m2_carry_prev;
        loc_m2_carry_prev = carry2; 
#else
        uint32_t old_acc = loc_acc1; loc_acc1 += (uint32_t)step;
        if (loc_acc1 < old_acc) total_correction = 1;
#endif

        int32_t final_div_fx4 = l_div_fx4 + total_correction;
        
        // Двусторонний аппаратный клемп 12-битного регистра CH_DIV (1.0 .. 255.15)
        if (__builtin_expect(final_div_fx4 < 16, 0))   final_div_fx4 = 16;
        if (__builtin_expect(final_div_fx4 > 4095, 0)) final_div_fx4 = 4095;

        *pwm_div_reg = (uint32_t)final_div_fx4;
#else
        dds_accumulator = loc_acc1; xorshift_state = loc_rand_state;
#ifdef VFO_USE_MASH2
        dds_accum_m2 = loc_acc2; m2_carry_prev = loc_m2_carry_prev;
#endif
        vfo_dither_step((uint32_t)l_step, (uint32_t)l_div_fx4);
        loc_acc1 = dds_accumulator; loc_rand_state = xorshift_state;
#ifdef VFO_USE_MASH2
        loc_acc2 = dds_accum_m2; loc_m2_carry_prev = m2_carry_prev;
#endif
#endif
    }
}

static void detach_peripheral_clock() {
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * 1000000, 48 * 1000000);
}

// === ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ ===
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz) {
    multicore_reset_core1(); 
    current_active_tone = VFO_TONE_NONE;
    xorshift_state = VFO_RAND_SEED_INIT + time_us_32(); 

    if (vfo_spin_lock == nullptr) {
        int lock_id = spin_lock_claim_unused(true);
        vfo_spin_lock = spin_lock_init(lock_id);
    }

    uint32_t best_fbdiv = 133, best_p1 = 6, best_p2 = 2;
    uint64_t clk_sys_target_hz = 133000000ULL;

#ifdef VFO_PLL_AUTOTUNE
    PllConfig optimal_pll = vfo_find_optimal_pll(base_freq_hz);
    best_fbdiv = optimal_pll.fbdiv;
    best_p1 = optimal_pll.p1;
    best_p2 = optimal_pll.p2;
    clk_sys_target_hz = optimal_pll.clk_sys_hz;
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

    gpio_set_function(VFO_OUTPUT_PIN, GPIO_FUNC_PWM);
    uint_slice_num = pwm_gpio_to_slice_num(VFO_OUTPUT_PIN);
    uint_pwm_chan = pwm_gpio_to_channel(VFO_OUTPUT_PIN);

    // СЕТКА ТОНОВ С ЗАЩИТОЙ ОТ FLOATING POINT ОКРУГЛЕНИЯ И ЧЕСТНЫМ STEP_HZ
    for (int i = 0; i < VFO_IFKP_TONES_COUNT; i++) {
        double exact_tone_freq = (double)base_freq_hz + ((double)i * step_hz);
        uint64_t tone_freq_mhz = (uint64_t)(exact_tone_freq * 1000.0);
        ifkp_tones[i] = calculate_raw_params_mhz(current_clk_sys_hz, tone_freq_mhz);
    }

    VfoParameters base_p = ifkp_tones[0];
    pwm_set_wrap(uint_slice_num, base_p.pwm_wrap);
    pwm_set_chan_level(uint_slice_num, uint_pwm_chan, base_p.pwm_wrap >> 1);
    pwm_set_clkdiv_int_frac(uint_slice_num, base_p.pwm_base_div_fx4 >> 4, base_p.pwm_base_div_fx4 & 0x0F);
    pwm_set_phase_correct(uint_slice_num, true);
    pwm_set_enabled(uint_slice_num, false); 

    vfo_set_tone_instant(0);

    tone_changed = true;
    multicore_launch_core1(vfo_core1_entry); 
}

/**
 * Мгновенная атомарная смена тона передачи
 */
void __not_in_flash_func(vfo_set_tone_instant)(uint8_t tone_index) {
    if (tone_index >= VFO_IFKP_TONES_COUNT) return; 
    if (tone_index == current_active_tone) return;

    VfoParameters t = ifkp_tones[tone_index];

    pwm_set_wrap(uint_slice_num, t.pwm_wrap);
    pwm_set_chan_level(uint_slice_num, uint_pwm_chan, t.pwm_wrap >> 1);

    uint32_t save = spin_lock_blocking(vfo_spin_lock);
    target_pwm_wrap     = t.pwm_wrap;
    target_pwm_base_div = t.pwm_base_div_fx4;
    dds_step            = t.dds_step; 
    tone_changed        = true; 
    spin_unlock(vfo_spin_lock, save);

    current_active_tone = tone_index;
}

void __not_in_flash_func(vfo_operation_set)(bool key_down) {
    pwm_set_enabled(uint_slice_num, key_down);
    dev_TX_state = key_down;
}

