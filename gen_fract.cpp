#include "gen_fract.h"

// ПОДКЛЮЧЕНИЕ НИЗКОУРОВНЕВОГО ЖЕЛЕЗА PICO SDK
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/resets.h"
#include "hardware/sync.h"

// при установке частоты используется дробный делитель PIO

static uint32_t current_tuned_frequency = 7050000;
volatile uint32_t sdr_live_sys_hz = 125000000;

PIO lo_pio = pio0;
uint lo_sm = 0;
uint pio_program_offset = 0; // Смещение программы в памяти PIO

// Граничные условия частот
const uint32_t MIN_FREQ = 100000;       // 100 кГц
const uint32_t MAX_FREQ = 33250000;     // 33.25 МГц
const uint32_t MIN_SYS_FREQ = 110000000; // 110 МГц
const uint32_t MAX_SYS_FREQ = 135000000; // 135 МГц (Жёсткий лимит без разгона)
const uint64_t BASE_XOSC_HZ = 12000000ULL;                 // Опорный кварц 12 МГц
const uint64_t CALIBRATED_XOSC_HZ = BASE_XOSC_HZ + 350ULL; // Физическая коррекция (+29.2 PPM)

// ==========================================
// PIO ПРОГРАММА (ПРАВКА: 2 такта на период простой частоты)
// ==========================================
const uint16_t nco_program_instructions[] = {
    (uint16_t)pio_encode_set(pio_pins, 0), // Шаг 0: Логический 0
    (uint16_t)pio_encode_set(pio_pins, 1), // Шаг 1: Логическая 1
};

const pio_program_t nco_program = {
    .instructions = nco_program_instructions,
    .length = 2,                          // Длина уменьшена до 2
    .origin = -1,
    .pio_version = 0,
};


// ==========================================
// ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ
// ==========================================
void fractGen_init() {
  Serial.print(F("[Система] Инициализация программного синтезатора частоты на пине "));
  Serial.println(PIN_I); // ПРАВКА: Убрали упоминание PIN_Q
  
  // Загружаем программу в память PIO и запоминаем её смещение
  pio_program_offset = pio_add_program(lo_pio, &nco_program);
  
  // Запускаем сканирующий матричный алгоритм для стартовой частоты 7.050 МГц
  set_pio_sdr_freq(current_tuned_frequency);
}


// ==========================================
// ПРОДВИНУТЫЙ СКАНИРУЮЩИЙ АЛГОРИТМ НАСТРОЙКИ PLL
// ==========================================
void set_pio_sdr_freq(uint32_t target_frequency_hz) {
    if (target_frequency_hz < MIN_FREQ)  target_frequency_hz = MIN_FREQ;
    if (target_frequency_hz > MAX_FREQ)  target_frequency_hz = MAX_FREQ;
    if (target_frequency_hz == current_tuned_frequency) return;
    //fractGen_OFF();

    uint64_t crystal_hz = CALIBRATED_XOSC_HZ;
    uint32_t best_refdiv = 1;
    uint32_t best_p1 = 4;
    uint32_t best_p2 = 2;
    uint32_t best_fbdiv_int = 100;
    uint32_t best_pio_div_fixed8 = 256;
    uint32_t final_sys_hz = 120000000; 
    
    uint64_t best_error_hz = 0xFFFFFFFFFFFFFFFFULL; 
    bool found_match = false;

    // ============================================================
    // БЛОК 1: ПОЛНЫЙ БЕЗЗНАКОВЫЙ МАТРИЧНЫЙ ПЕРЕБОР С КРЕМНИЕВЫМИ ФИЛЬТРАМИ
    // ============================================================
    
    // ПРЯМОЙ ОБХОД ДЛЯ ДВ/СВ ДИАПАЗОНОВ (Ниже 2 МГц)
    if (target_frequency_hz < 2000000ULL) {
        best_refdiv = 1; best_fbdiv_int = 80; best_p1 = 4; best_p2 = 2;
        final_sys_hz = 120000000ULL;
        // ПРАВКА: делитель pio_denom теперь равен частоте * 2 (две инструкции в цикле PIO)
        uint64_t pio_denom = (uint64_t)target_frequency_hz * 2ULL;
        best_pio_div_fixed8 = (final_sys_hz * 256ULL) / pio_denom;
    } 
    // СКВОЗНОЙ ПОИСК МИНИМУМА ОШИБКИ ДЛЯ КВ ДИАПАЗОНА (Выше 2 МГц)
    else {
        uint64_t crystal_hz = CALIBRATED_XOSC_HZ;
        // ПРАВКА: тактовая частота PIO теперь target * 2
        uint64_t target_pio_clk = (uint64_t)target_frequency_hz * 2ULL;
        
        uint64_t best_error_hz = 0xFFFFFFFFFFFFFFFFULL; 
        bool found_match = false;

        for (uint32_t p1 = 2; p1 <= 6; p1++) {
            for (uint32_t p2 = 1; p2 <= 2; p2++) {
                uint32_t pdiv_total = p1 * p2;
                
                for (uint32_t fbdiv_int = 30; fbdiv_int <= 150; fbdiv_int++) {
                    
                    uint64_t actual_vco = ((uint64_t)fbdiv_int * crystal_hz);
                    
                    if (actual_vco < 400000000ULL || actual_vco > 1200000000ULL) continue;
                    
                    uint64_t clk_sys_hz = actual_vco / (uint64_t)pdiv_total;
                    
                    if (clk_sys_hz < 100000000ULL || clk_sys_hz > 133000000ULL) continue;
                    
                    uint64_t pio_div_fixed8 = (clk_sys_hz * 256ULL) / target_pio_clk;
                    if (pio_div_fixed8 < 256ULL || pio_div_fixed8 > (65000ULL * 256ULL)) continue;

                    // Точный обратный расчет получающейся частоты гетеродина
                    // ПРАВКА: Изменено умножение с 64 на 128 из-за изменения базового шага частоты (2 вместо 4)
                    uint64_t actual_out = (clk_sys_hz * 128ULL) / pio_div_fixed8;
                    
                    uint64_t error_hz = 0;
                    if ((uint64_t)target_frequency_hz > actual_out) {
                        error_hz = (uint64_t)target_frequency_hz - actual_out;
                    } else {
                        error_hz = actual_out - (uint64_t)target_frequency_hz;
                    }

                    if (error_hz < best_error_hz) {
                        best_refdiv = 1;
                        best_error_hz = error_hz; 
                        best_fbdiv_int = fbdiv_int;
                        best_p1 = p1;
                        best_p2 = p2;
                        best_pio_div_fixed8 = (uint32_t)pio_div_fixed8;
                        final_sys_hz = (uint32_t)clk_sys_hz;
                        found_match = true;
                    }
                }
            }
        }
        if (!found_match) {
            best_refdiv = 1; best_fbdiv_int = 95; best_p1 = 5; best_p2 = 2; 
            final_sys_hz = 114000000ULL;
            best_pio_div_fixed8 = (final_sys_hz * 256ULL) / (target_pio_clk);
        }
    }


    // ============================================================
    // БЛОК 2: АППАРАТНАЯ КОММУТАЦИЯ ЖЕЛЕЗА
    // ============================================================
    uint32_t sm_mask = 1u << lo_sm;
    uint32_t ints_status = save_and_disable_interrupts();
    
    clock_configure(
        clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
        12000000,
        12000000
    );
    
    reset_block(RESETS_RESET_PLL_SYS_BITS);
    unreset_block_wait(RESETS_RESET_PLL_SYS_BITS);
    
    uint32_t vco_nominal_hz = (uint32_t)((CALIBRATED_XOSC_HZ * (uint64_t)best_fbdiv_int) / (uint64_t)best_refdiv);
    pll_init(pll_sys, best_refdiv, vco_nominal_hz, best_p1, best_p2);
    
    hw_write_masked(&clocks_hw->clk[clk_sys].ctrl,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS
    );
    hw_write_masked(&clocks_hw->clk[clk_sys].ctrl,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX << CLOCKS_CLK_SYS_CTRL_SRC_LSB,
        CLOCKS_CLK_SYS_CTRL_SRC_BITS
    );
    clocks_hw->clk[clk_sys].div = 0x100;

    uint32_t pll_sys_fbdiv_reg = *((volatile uint32_t*)(PLL_SYS_BASE + 0x08)) & 0xFFF;
    uint32_t pll_sys_prim_reg  = *((volatile uint32_t*)(PLL_SYS_BASE + 0x0c));
    uint32_t reg_pdiv1 = (pll_sys_prim_reg >> 16) & 0x7;
    uint32_t reg_pdiv2 = (pll_sys_prim_reg >> 12) & 0x7;
    uint64_t real_clk_sys_hz = (CALIBRATED_XOSC_HZ * (uint64_t)pll_sys_fbdiv_reg) / (uint64_t)(reg_pdiv1 * reg_pdiv2);

    // ПРАВКА: Финальный расчет под измененную тактовую частоту PIO (умножаем на 2 вместо 4)
    uint64_t pio_denom = (uint64_t)target_frequency_hz * 2ULL;
    uint64_t final_pio_div_fixed8 = (real_clk_sys_hz * 256ULL) / pio_denom;
    if (final_pio_div_fixed8 < 256ULL) final_pio_div_fixed8 = 256ULL;

    // ============================================================
    // БЛОК 3: КОНФИГУРИРОВАНИЕ И СТАРТ АВТОМАТА PIO
    // ============================================================
    pio_set_sm_mask_enabled(lo_pio, sm_mask, false);
    
    gpio_init(PIN_I);
    // ПРАВКА: PIN_Q больше не инициализируем и не переводим в OUTPUT
    pinMode(PIN_I, OUTPUT);
    digitalWrite(PIN_I, LOW);
    busy_wait_us(2); 

    pio_sm_config config = pio_get_default_sm_config();
    // ПРАВКА: Изменен wrap программы — теперь зацикливание идет с 0 на 1 инструкцию
    sm_config_set_wrap(&config, pio_program_offset + 0, pio_program_offset + 1);
    // ПРАВКА: Выделяем под set-пины только 1 пин (PIN_I) вместо 2
    sm_config_set_set_pins(&config, PIN_I, 1);

    uint16_t div_int = final_pio_div_fixed8 / 256;
    uint8_t div_frac = final_pio_div_fixed8 % 256;
    sm_config_set_clkdiv_int_frac(&config, div_int, div_frac);
    
    pio_sm_init(lo_pio, lo_sm, pio_program_offset, &config);
    pio_sm_clear_fifos(lo_pio, lo_sm);
    pio_sm_restart(lo_pio, lo_sm);
    
    pio_gpio_init(lo_pio, PIN_I); 
    // ПРАВКА: pio_gpio_init для PIN_Q удален
    // ПРАВКА: Направление пина выставляем только для 1 пина (PIN_I)
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, PIN_I, 1, true);
    pio_set_sm_mask_enabled(lo_pio, sm_mask, true);
    
    current_tuned_frequency = target_frequency_hz;
    restore_interrupts(ints_status);
}

/**
 * @brief Вычисляет реальную физическую частоту на выходе, основываясь на данных из регистров.
 * @return Реальная частота генерации в Герцах (uint32_t). Возвращает 0, если генерация выключена.
 */
uint32_t fractGen_get_real_frequency() {
    uint32_t sm_mask = 1u << lo_sm;
    
    // Если стейт-машина PIO сейчас остановлена, реальная частота на выходе равна нулю
    if ((lo_pio->ctrl & sm_mask) == 0) {
        return 0;
    }

    // 1. Чтение регистров PLL_SYS для вычисления частоты системной шины (clk_sys)
    uint32_t pll_sys_fbdiv_reg = *((volatile uint32_t*)(PLL_SYS_BASE + 0x08)) & 0xFFF;
    uint32_t pll_sys_prim_reg  = *((volatile uint32_t*)(PLL_SYS_BASE + 0x0c));
    
    uint32_t reg_pdiv1 = (pll_sys_prim_reg >> 16) & 0x7;
    uint32_t reg_pdiv2 = (pll_sys_prim_reg >> 12) & 0x7;
    
    // Предотвращаем деление на ноль в случае некорректного состояния регистров PLL
    if (reg_pdiv1 == 0 || reg_pdiv2 == 0) return 0;

    // Вычисляем точную clk_sys (в 64-битном беззнаковом целом для защиты от переполнения)
    uint64_t real_clk_sys_hz = (CALIBRATED_XOSC_HZ * (uint64_t)pll_sys_fbdiv_reg) / (uint64_t)(reg_pdiv1 * reg_pdiv2);

    // 2. Чтение текущего делителя частоты из регистров стейт-машины PIO
    uint32_t clkdiv_reg = lo_pio->sm[lo_sm].clkdiv;
    
    uint32_t div_int  = (clkdiv_reg >> 16) & 0xFFFF;
    uint32_t div_frac = (clkdiv_reg >> 8) & 0xFF;

    // Если целочисленный регистр пуст, аппаратная логика PIO выставляет делитель 1.0 (div_int = 1, div_frac = 0)
    if (div_int == 0) {
        div_int = 1;
        div_frac = 0;
    }

    // Восстанавливаем полный делитель PIO в формате fixed-point 16.8 (умноженный на 256)
    uint32_t pio_divider_fixed8 = (div_int << 8) | div_frac;

    // 3. Финальный расчет частоты сигнала без использования float
    // Базовая формула: F_out = clk_sys / (pio_divider / 2) -> F_out = (clk_sys * 2) / (pio_divider_fixed8 / 256)
    // Преобразуем к целочисленному виду: F_out = (clk_sys * 512) / pio_divider_fixed8
    // Деление на 2 для меандра учтено в множителе: 512 / 2 = 256
    uint64_t real_frequency_hz = (real_clk_sys_hz * 128ULL) / (uint64_t)pio_divider_fixed8;

    return (uint32_t)real_frequency_hz;
}




/**
 * @brief Быстро включает генерацию частоты.
 */
void fractGen_ON() {
    uint32_t sm_mask = 1u << lo_sm;
    // Запускаем конечный автомат PIO
    pio_set_sm_mask_enabled(lo_pio, sm_mask, true);
}

/**
 * @brief Быстро выключает генерацию частоты.
 */
void fractGen_OFF() {
    uint32_t sm_mask = 1u << lo_sm;
    // Мгновенно останавливаем автомат PIO
    pio_set_sm_mask_enabled(lo_pio, sm_mask, false);
    // Принудительно переводим пин в безопасное низкое состояние, 
    // чтобы в момент остановки на выходе не осталась логическая "1"
    gpio_put(PIN_I, false);
}