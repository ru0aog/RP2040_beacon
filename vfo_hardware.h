#ifndef VFO_HARDWARE_H
#define VFO_HARDWARE_H

#include <Arduino.h>

// Лимиты частотного плана
// vfo_hardware.h  
#define VFO_CLK_SYS_NOMINAL_HZ  133000000ULL  // электрический номинал без вольтмода  
#define VFO_CLK_SYS_MIN_HZ      100000000ULL  // нижняя граница поиска оптимума  
#define VFO_CLK_SYS_MAX_HZ      400000000ULL  // 410 - дефайн остаётся как АБСОЛЮТНЫЙ аппаратный барьер
#define PLL_MAX_HZ             3900000000ULL  // 3.9 - дефайн остаётся как АБСОЛЮТНЫЙ аппаратный барьер
#define VFO_FLASH_SCK_MAX_HZ     50000000ULL
#define VFO_THROTTLE_HI_C   80.0f   
#define VFO_THROTTLE_LO_C   60.0f

// рабочий лимит как переменные — min(дефайн, flash-значение)
extern uint32_t vfo_max_clk_sys_hz;   // из Flash, паспорт=133 МГц  
extern uint32_t vfo_max_pll_vco_hz;   // из Flash, паспорт=1600 МГц 

// Эффективный лимит поиска: flash-доказанный, но не выше аппаратного барьера  
static inline uint64_t vfo_effective_clk_max(void) {  
    uint64_t lim = (uint64_t)vfo_max_clk_sys_hz;  
    return (lim < VFO_CLK_SYS_MAX_HZ) ? lim : VFO_CLK_SYS_MAX_HZ;  
}  
static inline uint64_t vfo_effective_pll_max(void) {  
    uint64_t lim = (uint64_t)vfo_max_pll_vco_hz;  
    return (lim < PLL_MAX_HZ) ? lim : PLL_MAX_HZ;  
}

struct PllConfig {
    uint32_t fbdiv;
    uint32_t p1;
    uint32_t p2;
    uint64_t clk_sys_hz;
    uint32_t vsel;         
    bool is_oc;    
    uint32_t refdiv;        
};

// --- Архитектурное разделение профилей тактирования ---
extern PllConfig pll_nominal;     // Базовый гражданский режим (133 МГц)
extern PllConfig pll_overclock;   // Активный boost-профиль ТЕКУЩЕГО сеанса передачи
extern PllConfig pll_ceiling;     // Абсолютный жесткий потолок, доказанный OCTEST
extern volatile bool clk_boosted; 

struct VfoParameters {
    uint32_t pio_int;
    uint32_t pio_frac;
    uint32_t dds_step;
    uint32_t target_freq_chz; 
};




#define VFO_IFKP_TONES_COUNT 33            
#define VFO_TONE_NONE        255           
//#define VFO_CALIBRATED_XOSC_HZ 12000350ULL
// паспортный номинал кварца + относительная коррекция в ppb
// (1 ppm = 1000 ppb; ppb выбран для целочисленной точности без float)
#define VFO_XOSC_NOMINAL_HZ   12000000ULL   // кварц 12.000000 МГц  
#define VFO_XOSC_CORR_PPB     (+29167LL)    // +29.167 ppm -> 12000350 Гц
// вычисление скорректированной опоры
inline constexpr uint64_t VFO_CALIBRATED_XOSC_HZ =  
    (uint64_t)((int64_t)VFO_XOSC_NOMINAL_HZ +  
               (int64_t)VFO_XOSC_NOMINAL_HZ * VFO_XOSC_CORR_PPB / 1000000000LL);

extern VfoParameters __attribute__((section(".time_critical.ifkp_tones"))) ifkp_tones[VFO_IFKP_TONES_COUNT];
extern int pin_freq_out; 

#define VFO_OUTPUT_PIN       (pin_freq_out)

// Прототипы с аргументами по умолчанию
PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz, uint64_t max_clk_limit = 0); // 0 = auto

void vfo_hardware_init(unsigned int base_freq_hz, double step_hz);
void vfo_set_tone_instant(uint8_t tone_index);
void vfo_operation_set(bool key_down);
void vfo_clk_boost_enter(unsigned int target_freq_hz);
void vfo_clk_boost_exit(void);
void vfo_clk_thermal_guard(void);     
void vfo_find_max_stable_clock(void);
void vfo_test_pll_extreme_shurm(void);


// --- Энкодер перестройки частоты ---  
#define ENC_PIN_A            20   // канал A (фаза), INPUT_PULLUP  
#define ENC_PIN_B            21   // канал B (фаза), INPUT_PULLUP
#define VFO_ENC_SW_PIN       19        // кнопка (нажатие = замыкание на GND)  
#define VFO_ENC_TEST_FREQ_HZ 3600000UL // частота генерации по кнопке 
#define VFO_ENC_DEBOUNCE_MS  30     // щелчков на оборот (справочно, не критично)
  
// Шаг перестройки на один щелчок энкодера, Гц. Задаётся снаружи,  
// можно менять на лету (например, командой из консоли).  
extern volatile uint32_t vfo_tune_step_hz;  
// Текущая рабочая частота VFO, Гц — глобальная точка правды для ISR.  
extern volatile uint32_t vfo_current_freq_hz;  
// Накопленные тики энкодера со времени последней обработки.  
extern volatile int32_t  enc_pending_steps;  
  
void vfo_encoder_init(void);          // pinMode + attachInterrupt, вызывать из setup()  
void vfo_encoder_poll(void);          // обработка накопленных тиков, вызывать из loop()



#endif // VFO_HARDWARE_H
