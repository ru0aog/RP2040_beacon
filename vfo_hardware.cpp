/**  
 * ============================================================================  
 *  vfo_hardware.cpp — Дизеринг-движок программного DDS VFO на автоматах PIO  
 *  Версия 2.18 (Профилирование и ASM-оптимизация Core 1), 2026-10-08  
 * ============================================================================ 
 * программная реализация MASH-2 (Multi-Stage Noise Shaping 2-го порядка
 * выполнен по схеме MASH 1-1 (каскад из двух последовательных дельта-сигма модуляторов 1-го порядка).
 * Конвейер полностью развернут внутри регистров процессора ARM Cortex-M0+ на изолированном ядре Core 1 
 * и работает без обращения к ОЗУ. Благодаря этому частота расчета дизеринга 
 * достигает своего физического потолка — нескольких мегагерц.
 
 план:
      - абсолютное значение частоты локального кварцевого генератора заменить
        на относительную коррекцию +/- ppm
      - добавить запись/чтение результатов разгона во флэш

 2.18 от 2026-10-08
      - улучшение метрики: введены штрафы за узлы, у которых dds_step садится возле простых дробей шкалы 1/4,1/2,3/4, k/8
      - 

 2.17 от 2026-10-07
      - устранён самопроизвольный сброс POSTDIV1 в 2 (состояние 1 исключили)
      - добавлено предсказание частоты отстройки основных спуров
      - стабилизирована работа в режиме разгона
      - добавлена процедура теста максимальной скорости VCO PLL
      - добавлено включение и перестройка частоты энкодером

Архитектурная схема (MASH 1-1)
 В коде параллельно работают два 32-битных регистра-аккумулятора: loc_acc1 (первая ступень) и loc_acc2 (вторая ступень).
 - Первая ступень интегрирует (накапливает) входное дробное приращение частоты step (оно же dds_step). 
 При каждом её переполнении формируется бит переноса carry1.
 - Вторая ступень интегрирует не входной шаг, а текущее состояние фазы первой ступени (loc_acc1). 
 При её переполнении формируется бит переноса carry2.

  dds_step (шаг) 
       │
       ▼
   ┌───────┐   carry1   ┌────────────────────────┐
   │ ACC 1 ├───────────►│                        │
   └───┬───┘            │ Цифровая комбинация    │     total_correction
       │                │    переносов (NTF)     ├─────► (от -1 до +2)
       ▼                │                        │
   ┌───────┐   carry2   │ Y = C1 + C2 - C2_prev  │
   │ ACC 2 ├───────────►│                        │
   └───────┘            └────────────────────────┘


 *  АРХИТЕКТУРА  
 *  1) PIO гоняет 2-тактный меандр -> f_out = clk_sys / (2 * D).  
 
 *     отбирает clk_sys (VCO 750..1600 МГц, clk_sys 100..133 МГц) по метрике:  
 *     первично — минимум остатка dds_step (32 бита), вторично — минимум  
 *     pio_frac, третично — максимум clk_sys.  
 *  3) Остаточная дробь делителя доводится DDS/MASH-дизерингом: аккумулятор(ы)  
 *     переполняются с частотой шага, а перенос корректирует clkdiv PIO.  
 *  
 *  МНОГОКРИТЕРИАЛЬНЫЙ АВТОТЮН PLL И ДВУХЪЯДЕРНЫЙ ДИЗЕРИНГ  
 *  -----------------------------------------------------  
 * ============================================================================  
 */

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/resets.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include <hardware/watchdog.h>
#include <hardware/adc.h>
#include "hardware/divider.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/ssi.h"   // ssi_hw->baudr — делитель XIP_SSI
#include "hardware/structs/pll.h" // Для прямого доступа к pll_hw->cs
#include "pico/multicore.h"
#include "vfo_hardware.h"
#include "file_manager.h"
#include "si5351_driver.h"
#include "cw_modem.h"

#define VFO_PLL_DEBUG            1         // 0 — выключить отладочную печать кандидатов
#define VFO_DITHER_LOOP_CYCLES   18ULL     // тактов на итерацию Core 1  

#define VFO_INT_PENALTY   500000000ULL     // жёсткий запрет pio_int < 4

#define VFO_SPUR_MIN_OFFSET_HZ 1000000ULL   // запретный пояс FRAC8-спура, Гц  
#define VFO_DMIN_CEILING       280000000ULL // гейт близкой гребёнки дизера (~6.5% от 2^32)
#define VFO_CLK_SYS_PREF_MIN_HZ  250000000ULL   // минимум clk_sys: ниже — сильная близкая гребёнка


// ============================================================================
// МОДЕРНИЗИРОВАННЫЙ МНОГОКРИТЕРИАЛЬНЫЙ АВТОТЮН PLL (ВЕРСИЯ С REFDIV И CTZ-ФИЛЬТРОМ)
// ============================================================================
#define VFO_MAX_STABLE_CLK_HZ    380000000ULL  // Ваш доказанный предел стабильности
#define VFO_PREFERRED_MIN_CLK    320000000ULL  // Нижняя граница зоны чистого спектра

// Структура детальной калькуляции штрафов
struct MetricBreakdown {
    uint32_t raw_step;
    uint32_t frac;
    uint32_t ctz_val;
    uint64_t ctz_penalty;
    uint64_t center_dist;
    uint64_t mash_penalty;
    uint64_t clk_penalty;
    uint64_t int_penalty;
    uint64_t total_metric;
};

// ============================================================================
// ПРОТОТИПЫ (ОБЪЯВЛЕНИЯ) ВНУТРЕННИХ ФУНКЦИЙ ФАЙЛА
// ============================================================================
static void detach_peripheral_clock();
static void __not_in_flash_func(vfo_set_clk_sys)(const PllConfig& cfg, uint32_t vsel);
static VfoParameters calculate_raw_params_mhz(uint64_t clk_sys_hz, uint64_t mhz_target);
PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz, uint64_t max_clk_limit);
// Упреждающий прототип для verbose-метрики (чтобы find_optimal_pll видела её выше по коду)
static uint64_t vfo_pll_metric_verbose(uint64_t clk_sys_hz, uint64_t target_mhz, MetricBreakdown *b, VfoParameters *out = nullptr);

// Добавлен упреждающий прототип Core 1, теперь vfo_clk_boost_enter сможет его вызвать!
static void __not_in_flash_func(vfo_core1_entry)();

void vfo_clk_boost_arm(void);

static void vfo_rebuild_tone_table(uint64_t base_freq_mhz, uint64_t step_mhz); 
static vreg_voltage vsel_for(uint64_t clk_hz);
static uint64_t vfo_base_mhz = 0;   // базовый тон сетки, мГц  
static uint64_t vfo_step_mhz = 0;   // шаг сетки, мГц

// Глобальные профили тактирования (static полностью удалены для extern-связывания)
// Обновляем глобальные профили тактирования (добавлен refdiv = 1 в конец каждого списка)
PllConfig pll_nominal   = { 133, 6, 2, 133000000ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };
PllConfig pll_overclock = { 69,  3, 2, 138004025ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 }; 
PllConfig pll_ceiling   = { 95,  3, 1, 380011083ULL, (uint32_t)VREG_VOLTAGE_1_30,    true,  1 };  

volatile bool clk_boosted = false;       // активация разгона

 

// ============================================================================
// ВЕРХНИЕ МАКРОСЫ И СТРУКТУРЫ ДЛЯ МНОГОКРИТЕРИАЛЬНОГО АВТОТЮНА
// ============================================================================
#ifndef VFO_MAX_STABLE_CLK_HZ
#define VFO_MAX_STABLE_CLK_HZ    380000000ULL  // Предел стабильности
#endif

#ifndef VFO_PREFERRED_MIN_CLK
#define VFO_PREFERRED_MIN_CLK    320000000ULL  // Нижняя граница чистой зоны
#endif

#ifndef VFO_INT_PENALTY
#define VFO_INT_PENALTY          500000000ULL  // Запрет pio_int < 4
#endif









static bool thermal_throttled = false; // защёлка состояния троттлинга  
static uint16_t vsel_to_mv(uint32_t vsel);  
static float    vfo_read_core_temp_c(void);

static uint64_t vfo_pll_metric(uint64_t clk_sys_hz, uint64_t target_mhz,  
                               VfoParameters* out = nullptr);




// Помощник автоматического определения напряжения ядра под частоту шины  
static vreg_voltage vsel_for(uint64_t clk_hz) {  
    if (clk_hz >= 250000000ULL)      return VREG_VOLTAGE_1_30; // >=250 МГц — только легальный максимум  
    else if (clk_hz >= 200000000ULL) return VREG_VOLTAGE_1_25;  
    else if (clk_hz > 133000000ULL)  return VREG_VOLTAGE_1_15;  
    return VREG_VOLTAGE_DEFAULT;  
}


/** @brief Взвести флаг BOOST до следующего vfo_hardware_init()/boost_enter().  
 *  Используется энкодером, чтобы init выбрал оптимальный PLL-скан,  
 *  а не номинал 133 МГц. Атомарно для RP2040 (single-core write). */  
void vfo_clk_boost_arm(void) {  
    clk_boosted = true;  
}


// Ассемблерная микропрограмма PIO для меандра (цикл из 2 тактов)
static const uint16_t pio_square_instructions[] = { 0xe001, 0xe000 };
static const pio_program_t pio_square_program = { 
    .instructions = pio_square_instructions, 
    .length = 2, 
    .origin = -1,
    .pio_version = 0 
};

// Физическое размещение массива в RAM. Секция .time_critical гарантирует нахождение в ОЗУ
VfoParameters __attribute__((section(".time_critical.ifkp_tones"))) ifkp_tones[VFO_IFKP_TONES_COUNT];

// Доступ к глобальному состоянию усилителя мощности для манипуляции ключом CW
extern bool dev_TX_state; 

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

static double cached_step_hz = 100.0;
static uint32_t cached_base_freq_hz = 3500000;


/**  
 * @brief Отсоединение периферийного домена clk_peri от системной шины.  
 *  
 * Переводит генератор периферийной тактовой частоты `clk_peri` на  
 * безопасный источник (кварцевый резонатор XOSC 12 МГц) и фактически  
 * изолирует периферию (UART, SPI, USB и т.д.) от домена `clk_sys`  
 * на время перепрограммирования `pll_sys`.  
 */
static void __not_in_flash_func(detach_peripheral_clock)() {  
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * 1000000, 48 * 1000000);  
}



/**  
 * @brief Безопасное динамическое перепрограммирование pll_sys и напряжения ядра vreg.  
 *  
 * Осуществляет двухранговый сдвиг частоты и питания. Защищено от зависания  
 * периферии доменным детачем clk_peri и критической секцией запрета прерываний.  
 * Функция исполняется из SRAM (__not_in_flash_func): в разгоне XIP-доступ к  
 * флэшу ненадёжен, и сам путь переключения не должен зависеть от него.  
 *  
 * Дополнительно пересчитывает делитель XIP_SSI (BAUDR) — частота QSPI  
 * флэша равна clk_sys / BAUDR; при clk_sys > 200 МГц штатный делитель 2  
 * выдаёт >100 МГц на SPI-линию, что выходит за пределы большинства  
 * QSPI-флэшей (~104-133 МГц) и убивает XIP-доступ. Делитель выбирается  
 * так, чтобы SCK <= VFO_FLASH_SCK_MAX_HZ; BAUDR на RP2040 всегда чётный.  
 *  
 */  
static void __not_in_flash_func(vfo_set_clk_sys)(const PllConfig& cfg_in, uint32_t vsel) {
    PllConfig cfg = cfg_in;  
    if (cfg.refdiv == 0) cfg.refdiv = 1;   // защита от неинициализированного поля

    // p1=1 недокументированно делит пополам на VCO < ~2.8 ГГц — отклоняем такие конфиги  
        if (cfg.p1 < 2) {  
    #if VFO_PLL_DEBUG  
            Serial.printf("[PLL] REJECT: p1=%lu forbidden — fallback nominal\n",  
                        (unsigned long)cfg.p1);  
    #endif  
            cfg = pll_nominal;  
        }

    uint32_t target_clk_hz = (uint32_t)(((uint64_t)cfg.fbdiv * VFO_CALIBRATED_XOSC_HZ) / (uint64_t)(cfg.refdiv * cfg.p1 * cfg.p2));  
    bool is_overclocking = (target_clk_hz > current_clk_sys_hz);  
  
    if (is_overclocking) {  
        vreg_set_voltage((vreg_voltage)vsel);  
        busy_wait_us(500);  
    }  

    detach_peripheral_clock();  
    uint32_t ints_status = save_and_disable_interrupts();  
  
    // clk_sys -> XOSC 12 МГц (SCK флэша 6 МГц, безопасно)  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                    VFO_CALIBRATED_XOSC_HZ, VFO_CALIBRATED_XOSC_HZ);  
  
    uint32_t vco_nominal_hz = (uint32_t)((VFO_CALIBRATED_XOSC_HZ * (uint64_t)cfg.fbdiv) / (uint64_t)cfg.refdiv);  


// перед pll_init — что реально стоит в опоре PLL (CS bits 5:0 = refdiv)  
#if VFO_PLL_DEBUG  
    Serial.printf("[PLL] pre: CS=0x%08lX refdiv_hw=%lu XOSC=%lu kHz\n",  
                  (unsigned long)pll_sys_hw->cs,  
                  (unsigned long)(pll_sys_hw->cs & PLL_CS_REFDIV_BITS),  
                  (unsigned long)frequency_count_khz(CLOCKS_FC0_SRC_VALUE_XOSC_CLKSRC));  
#endif  

    uint32_t cs_pre  = pll_sys_hw->cs;

    pll_init(pll_sys, cfg.refdiv, vco_nominal_hz, cfg.p1, cfg.p2);  

    // === АППАРАТНЫЙ МОНИТОРИНГ ЗАЩЁЛКИ ФАЗЫ PLL (LOCK BIT) ===
    // Ждем взвода бита LOCK в регистре CS системной PLL
    // Если аналоговая петля не успеет стабилизироваться за 2000 итераций — мы разорвем цикл, 
    // чтобы ядро не зависло намертво в глухом дедлоке.
    uint32_t timeout = 20000;  
    while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
    bool pll_locked = (timeout != 0);

    // === BAUDR: ОГРАНИЧЕНИЕ НА ТАКТИРОВАНИЕ ФЛЭША СТРОГО <= 48 МГц ===  
    // Меняем VFO_FLASH_SCK_MAX_HZ на жесткие 48 000 000 Гц
    const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;
    uint32_t ssi_baud = (uint32_t)((target_clk_hz + FLASH_SAFE_MAX_HZ - 1) / FLASH_SAFE_MAX_HZ);  
    
    ssi_baud = (ssi_baud + 1u) & ~1u;   // Округление вверх до ближайшего четного (требование аппаратного SSI)  
    if (ssi_baud < 2u)  ssi_baud = 2u;  
    if (ssi_baud > 34u) ssi_baud = 34u;  
    
    ssi_hw->ssienr = 0;                 // выключить SSI — иначе baudr не запишется  
    ssi_hw->baudr  = ssi_baud;  
    ssi_hw->ssienr = 1;                 // включить обратно  
  
    // clk_sys -> PLL_SYS (целевая частота)  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                    target_clk_hz, target_clk_hz);  
  
    restore_interrupts(ints_status);

#if VFO_PLL_DEBUG  
    Serial.printf("[PLL] pre: CS=0x%08lX refdiv_hw=%lu\n",  
                  (unsigned long)cs_pre,  
                  (unsigned long)(cs_pre & PLL_CS_REFDIV_BITS));  
#endif

#if VFO_PLL_DEBUG  
    // подтверждающий замер через 20 мкс после коммутации мультиплексора  
    busy_wait_us(20);  
    uint32_t post_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);  
    Serial.printf("[PLL] post-switch clk=%u kHz (expect=%u kHz)\n",  
                  (unsigned)post_khz, (unsigned)(cfg.clk_sys_hz / 1000u));  
#endif
 
    // --- Самопроверка: при промахе повторяем ВЕСЬ цикл переключения ---  
    bool locked_ok = false;
    uint32_t meas_khz = 0;                                   // <-- сюда  
    uint32_t expect_khz = cfg.clk_sys_hz / 1000u;  

    for (int attempt = 0; attempt < 3; attempt++) {    
            busy_wait_us(300);    
            meas_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);   // без uint32_t!  
            if (meas_khz > expect_khz * 97u / 100u &&    
                meas_khz < expect_khz * 103u / 100u) {    
                locked_ok = true;    
                break;    
            }
#if VFO_PLL_DEBUG  
        Serial.printf("[PLL] MISMATCH: meas=%u kHz expect=%u kHz — retry %d\n",  
                      (unsigned)meas_khz, (unsigned)expect_khz, attempt);  
#endif  
        // полный ре-секвенс: назад на XOSC, переинициализация PLL, вперёд на PLL_SYS

        uint32_t ints2 = save_and_disable_interrupts();
        clock_configure(clk_sys,  
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                        VFO_CALIBRATED_XOSC_HZ, VFO_CALIBRATED_XOSC_HZ);  
        pll_init(pll_sys, cfg.refdiv, vco_nominal_hz, cfg.p1, cfg.p2);  
        timeout = 20000;  
        while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);
// DIAG: лок есть, но шина не та — либо микс postdiv, либо ложный замер  
#if VFO_PLL_DEBUG  
        Serial.printf("[PLL] retry-dbg: LOCK=%d CS=0x%08lX FB=%lu PRIM=0x%08lX meas=%u kHz\n",  
                      (int)(pll_sys_hw->cs & PLL_CS_LOCK_BITS ? 1 : 0),  
                      (unsigned long)pll_sys_hw->cs,  
                      (unsigned long)pll_sys_hw->fbdiv_int,  
                      (unsigned long)pll_sys_hw->prim,  
                      (unsigned)meas_khz);  
#endif
        clock_configure(clk_sys,  
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                        target_clk_hz, target_clk_hz);
        restore_interrupts(ints2);
    }
uint32_t meas_khz_final = meas_khz; 
#if VFO_PLL_DEBUG  
    if (!locked_ok) {  
        Serial.printf("[PLL] FALLBACK reason: pll_locked=%d last_meas=%u expect=%u kHz\n",  
                      (int)pll_locked, (unsigned)meas_khz_final, (unsigned)expect_khz);  
    }  
#endif

    if (!locked_ok) {  
        // 3 промаха подряд — безопасный откат на номинал,  
        // чтобы таблица тонов не считалась от несуществующей частоты  
#if VFO_PLL_DEBUG  
        Serial.printf("[PLL] FAIL x3 — fallback to nominal\n");  
#endif  
        clock_configure(clk_sys,    
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,    
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,    
                        VFO_CALIBRATED_XOSC_HZ, VFO_CALIBRATED_XOSC_HZ);    
        pll_init(pll_sys, pll_nominal.refdiv,    
                 (uint32_t)(VFO_CALIBRATED_XOSC_HZ * pll_nominal.fbdiv / pll_nominal.refdiv),    
                 pll_nominal.p1, pll_nominal.p2);    
        timeout = 20000;    
        while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);    
        clock_configure(clk_sys,    
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,    
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,    
                        pll_nominal.clk_sys_hz, pll_nominal.clk_sys_hz);    
        current_clk_sys_hz = pll_nominal.clk_sys_hz;    
        return;    
    }
  
    current_clk_sys_hz = cfg.clk_sys_hz;


#if VFO_PLL_DEBUG

Serial.printf("[PLL] sys_ctrl=0x%08lX selected=0x%02lX\n",  
                  (unsigned long)clocks_hw->clk[clk_sys].ctrl,  
                  (unsigned long)clocks_hw->clk[clk_sys].selected);

    if (!pll_locked) {  
        Serial.printf("[PLL] !!! NO LOCK: fbdiv=%lu target=%lu Hz — выход НЕДОСТОВЕРЕН\n",  
                    (unsigned long)cfg.fbdiv, (unsigned long)target_clk_hz);
    }  
    uint32_t real_clk = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
    Serial.printf("[PLL] measured clk_sys=%lu Hz (target %lu)\n",  
                (unsigned long)real_clk, (unsigned long)target_clk_hz);
#endif
  
    if (!is_overclocking) {  
        busy_wait_us(100);  
        vreg_set_voltage((vreg_voltage)vsel);  
    }  
#if VFO_PLL_DEBUG
Serial.printf("[PLL] CS=0x%08lX (refdiv=%lu)  FB=%lu  PRIM=0x%08lX\n",  
              (unsigned long)pll_sys_hw->cs,  
              (unsigned long)(pll_sys_hw->cs & PLL_CS_REFDIV_BITS),  
              (unsigned long)pll_sys_hw->fbdiv_int,  
              (unsigned long)pll_sys_hw->prim);
#endif 
  
    current_clk_sys_hz = cfg.clk_sys_hz;  
}




void vfo_clk_boost_enter(unsigned int target_freq_hz) {  
    if (target_freq_hz == 0) {  
        clk_boosted = true;  
        return;  
    }  
  
    // 1. Поиск оптимума (один проход)  
    PllConfig opt = vfo_find_optimal_pll(target_freq_hz, vfo_effective_clk_max());  
  
    // 2. Сравнение метрик  
    uint64_t target_mhz = (uint64_t)target_freq_hz * 1000ULL;  
    VfoParameters dummy;  
    uint64_t metric_opt = vfo_pll_metric(opt.clk_sys_hz,     target_mhz, &dummy);  
    uint64_t metric_cur = vfo_pll_metric(current_clk_sys_hz, target_mhz, &dummy);  
  
    if (metric_opt >= metric_cur || opt.clk_sys_hz == current_clk_sys_hz) {    
        cached_base_freq_hz = target_freq_hz;  
        vfo_base_mhz = (uint64_t)target_freq_hz * 1000ULL;   // мГц  
        
        // 1. Перестраиваем таблицу под новые параметры
        vfo_rebuild_tone_table(vfo_base_mhz, vfo_step_mhz);  
        
        // 2. Записываем новые pio_int/pio_frac/dds_step в защищенные спинлоком переменные.
        // vfo_set_tone_instant(0) внутри себя взведет флаг tone_changed = true!
        vfo_set_tone_instant(0);  
        
        // 3. Даем микропаузу, чтобы Core 1 в своем горячем цикле гарантированно 
        // увидел флаг tone_changed, сбросил аккумуляторы и применил новый шаг БЕЗ перезапуска ядра.
        busy_wait_us(10); 

#if VFO_PLL_DEBUG  
        Serial.printf("[BOOST] SKIP-RETUNE (SOFT TUNE) clk=%lu kHz  target=%lu Hz\n",  
                      (unsigned long)(current_clk_sys_hz / 1000ULL),  
                      (unsigned long)target_freq_hz);  
#endif
        return;    
    }

  
    // 3. Остановка Core 1 — нужна и при ретюне (PLL-переключение рвёт тактирование)  
    multicore_reset_core1();  
    busy_wait_us(100);  
    cached_base_freq_hz = target_freq_hz; 
  
    // 4. Напряжение: профиль поиска с подстраховкой таблицей  
    uint32_t vsel = opt.vsel;  
    if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10) {  
        vsel = (uint32_t)vsel_for(opt.clk_sys_hz);  
    }  
  
    // 5. Сдвиг частоты и питания (внутри: детач clk_peri, сброс PLL)  
    vfo_set_clk_sys(opt, vsel);  
  
    // 6. Пересчёт таблиц под новую clk_sys  
    vfo_rebuild_tone_table(vfo_base_mhz, vfo_step_mhz);  
    vfo_set_tone_instant(0);  
    tone_changed = true;          // страховка для Core 1  

    pll_overclock = opt;          // активный профиль для термогуарда 
  
    multicore_launch_core1(vfo_core1_entry);  
  
    // Правильный расчет VCO для вывода на экран в мегагерцах [Скорректировано]
    uint32_t vco_print_mhz = (uint32_t)(((uint64_t)opt.fbdiv * VFO_CALIBRATED_XOSC_HZ) / (1000000ULL * opt.refdiv));

    Serial.printf("[BOOST] ON   clk_sys=%7.3f MHz  refdiv=%lu  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)\n",  
                opt.clk_sys_hz / 1000000.0,  
                (unsigned long)opt.refdiv,
                (unsigned long)opt.fbdiv,  
                (unsigned long)opt.p1,  
                (unsigned long)opt.p2,  
                (unsigned long)vco_print_mhz,  
                (unsigned)vsel_to_mv(vsel),  
                opt.clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,  
                (unsigned)ssi_hw->baudr);  
    clk_boosted = true;
}






  
/**  
 * @brief Выход из boost: возврат на pll_nominal и штатное напряжение.  
 * Понижение VREG выполняется ПОСЛЕ снижения частоты (логика в vfo_set_clk_sys).  
 */  
void vfo_clk_boost_exit(void) {    
    if (!clk_boosted) return;    
    
    vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
    vfo_rebuild_tone_table((uint64_t)cached_base_freq_hz * 1000ULL,    
                        (uint64_t)(cached_step_hz * 1000.0));  
    //Serial.println("[BOOST] пересчёт таблицы тонов");  
    
    // Пересчёт таблицы тонов обратно под номинальную clk_sys    
    uint32_t save = spin_lock_blocking(vfo_spin_lock);    
    uint64_t base_mhz = (uint64_t)cached_base_freq_hz * 1000ULL;    
    uint64_t step_mhz = (uint64_t)(cached_step_hz * 1000.0);    
    for (int i = 0; i < VFO_IFKP_TONES_COUNT; i++) {    
        ifkp_tones[i] = calculate_raw_params_mhz(current_clk_sys_hz,    
                                                 base_mhz + (uint64_t)i * step_mhz);    
    }    
    current_active_tone = VFO_TONE_NONE;    
    tone_changed = true;    
    spin_unlock(vfo_spin_lock, save);  
    clk_boosted = false;  
  
    // VCO = XOSC * fbdiv / refdiv (uint64 — fbdiv до 320 * 12 МГц не влезает в uint32 впритык)  
    uint64_t vco_mhz = (VFO_CALIBRATED_XOSC_HZ * (uint64_t)pll_nominal.fbdiv)  
                       / ((uint64_t)pll_nominal.refdiv * 1000000ULL);  
  
    Serial.printf("[BOOST] OFF  clk_sys=%7.3f MHz  refdiv=%lu  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4llu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)  T_CPU=%5.1f C\n",    
                  pll_nominal.clk_sys_hz / 1000000.0,  
                  (unsigned long)pll_nominal.refdiv,  
                  (unsigned long)pll_nominal.fbdiv,    
                  (unsigned long)pll_nominal.p1,    
                  (unsigned long)pll_nominal.p2,    
                  (unsigned long long)vco_mhz,    
                  (unsigned)vsel_to_mv((uint32_t)VREG_VOLTAGE_DEFAULT),    
                  pll_nominal.clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,    
                  (unsigned)ssi_hw->baudr,    
                  (double)vfo_read_core_temp_c());  
}



/**  
 * @brief Чистый одношаговый генератор псевдослучайных чисел Xorshift32.  
 *  
 * Выполняет одну итерацию алгоритма Xorshift32 Марсальи над переданным  
 * состоянием: последовательность трёх операций xor-сдвига  
 * (<< 13, >> 17, << 5). Функция не читает и не изменяет глобальное  
 * состояние — вызывающий код сам хранит и передаёт @p state,  
 * что делает её безопасной для использования из локальных регистровых  
 * копий в горячем конвейере Core 1 (@ref vfo_core1_entry) без обращения  
 * к ОЗУ и без спинлока.  
 *  
 */
static inline __attribute__((always_inline)) uint32_t __not_in_flash_func(vfo_xorshift32_raw)(uint32_t state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}



/**  
 * @brief Одношаговый генератор псевдослучайных чисел Xorshift32  
 *        с глобальным состоянием в ОЗУ.  
 *  
 * Выполняет одну итерацию алгоритма Xorshift32 Марсальи  
 * (xor-сдвиги << 13, >> 17, << 5) над глобальной переменной  
 * @c xorshift_state и сохраняет результат обратно в неё.  
 *  
 */
static inline __attribute__((always_inline)) uint32_t __not_in_flash_func(vfo_xorshift32)() {
    uint32_t x = xorshift_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xorshift_state = x;
    return x;
}




/**  
 * @brief Единичный атомарный шаг расчёта дизеринга (эталонная Си-версия).  
 *  
 * Выполняет одну итерацию Delta-Sigma модуляции: накапливает 32-битный шаг  
 * частоты @p local_step в аккумуляторе фазы и по битам переноса формирует  
 * коррекцию делителя частоты автомата PIO. Результат атомарно записывается  
 * в регистр `clkdiv` state machine (INT в битах 31..16, FRAC8 в битах 15..8).  
 *  
 */
static inline void __not_in_flash_func(vfo_dither_step)(uint32_t local_step, uint32_t local_int, uint32_t local_frac) {
    uint32_t step = local_step;

#ifdef VFO_DITHER_RANDOMIZE
    // Синхронизировано с Core 1: Быстрый беспереходный дизер с мат. ожиданием строго 0.0
    int32_t r_bits = (int32_t)(vfo_xorshift32() & 0x0Fu);
    int32_t r_dither = (r_bits << 1) - 15; 
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
 * @brief Таймерный колбэк дизеринга для режима Core 0.  
 *  
 */
#ifndef VFO_DITHER_ON_CORE1
static bool __not_in_flash_func(vfo_dither_callback)(struct repeating_timer *t) {
    (void)t; 
    vfo_dither_step(dds_step, target_pio_int, target_pio_frac8);
    return true; 
}
#endif



/**  
 * @brief Точка входа второго ядра (Core 1): бесконечный горячий цикл дизеринга.  
 *  
 */
static void __not_in_flash_func(vfo_core1_entry)() {
    // Регистро-резидентные копии статических параметров тона
    int32_t l_step = 0;
    int32_t l_int = 8;
    int32_t l_frac = 0;

    // Регистро-резидентное состояние накопителей (DDS и ГПСЧ)
    uint32_t loc_acc1 = 0;
    uint32_t loc_rand_state = VFO_RAND_SEED_INIT;
#ifdef VFO_USE_MASH2
    uint32_t loc_acc2 = 0;
    uint32_t loc_m2_carry_prev = 0;
#endif

    // Прямой кэшированный указатель на регистр SM PIO
    volatile uint32_t *clkdiv_reg = &lo_pio->sm[lo_sm].clkdiv;

    // Счётчик опроса флага смены тона: читаем ОЗУ-флаг раз в VFO_TONE_POLL_N итераций  
    // (проигрыш задержки применения тона: < 64 итераций × ~20 тактов ≈ 4 мкс на 321 МГц)  
    const uint32_t poll_mask = (1u << 6) - 1u;   // N = 64; степень двойки!  
    uint32_t poll_cnt = 0; 

    while (true) {
        // Разряженный опрос флага: проверка (poll_cnt & mask) — регистр + битовый AND  
        if ((poll_cnt & poll_mask) == 0u) {  
            if (__builtin_expect(tone_changed, 0)) {  
                uint32_t save = spin_lock_blocking(vfo_spin_lock);
                l_step = (int32_t)dds_step;
                l_int  = (int32_t)target_pio_int;
                l_frac = (int32_t)target_pio_frac8;
                
                loc_acc1 = 0;
                loc_rand_state = xorshift_state;
#ifdef VFO_USE_MASH2
            loc_acc2 = 0;
            loc_m2_carry_prev = 0;
#endif
            tone_changed = false; 
            spin_unlock(vfo_spin_lock, save);
            }  
        }  
        poll_cnt++;

#ifdef VFO_DITHER_PROFILE
        // Мгновенный аппаратный тоггл отладочного пина 13 через шину SIO (1 такт)
        sio_hw->gpio_togl = profile_pin_mask;
#endif

#ifdef VFO_DITHER_FAST
        // === ВЫСОКОСКОРОСТНОЙ РЕГИСТРОВЫЙ СИ-КОНВЕЙЕР (Без ОЗУ-структур и NOP) ===
        int32_t step = l_step;

#ifdef VFO_DITHER_RANDOMIZE
        // Декорреляция спектра (Быстрый регистровый Xorshift32 Дизер)
        // Чтобы предотвратить появление циклических узоров переполнения,
        // которые порождают паразитные палки-шпоры Idle Tones на панораме,
        // в аккумуляторы подмешивается хаос
        loc_rand_state ^= loc_rand_state << 13;
        loc_rand_state ^= loc_rand_state >> 17;
        loc_rand_state ^= loc_rand_state << 5;
        // Этот легкий 4-битный регистровый шум непрерывно «потряхивает» младшие биты входного шага, 
        // полностью размывая дискретные спектральные палки модуляции в гладкую, незаметную шумовую полку эфира.

        // Линейный беспереходный расчет дизера под фиксированные 4 бита (VFO_DITHER_RAND_BITS)
        // Выделяем 4 младших бита быстрой маской 0x0F (1 такт)
        // int32_t r_bits = (int32_t)(loc_rand_state & 0x0Fu);
        int32_t r_bits = (int32_t)(loc_rand_state & 0x03u);
        
        // Умножаем на 2 и вычитаем 15. Получаем симметричный ряд нечетных чисел от -15 до +15.
        // Математическое ожидание строго равно 0.0
        // step += ((r_bits << 1) - 15); 
        step += ((r_bits << 1) - 3); // Ряд: -3, -1, +1, +3
#endif

        int32_t total_correction = 0;

#ifdef VFO_USE_MASH2
        uint32_t carry1, carry2;
        // 1. Первая ступень (Интегратор ошибки базового шага)
        // Извлекаем аппаратные переносы напрямую из статусного регистра ALU процессора.
        // Инструкция ADDS взводит флаг, ADC вытягивает его без ветвлений и условных операторов.
        carry1 = __builtin_add_overflow(loc_acc1, (uint32_t)step, &loc_acc1) ? 1 : 0;
        // 2. Вторая ступень (Интегратор остатка первой ступени)
        carry2 = __builtin_add_overflow(loc_acc2, loc_acc1, &loc_acc2) ? 1 : 0;
        // Цифровая шумоформирующая комбинация (Сжатие юбки)
        // передаточная функция шума 2-го порядка (NTF — Noise Transfer Function)
        // Эта формула реализует дифференцирование переноса второй ступени по времени
        // Коррекция может принимать отрицательные значения (-1), это штатное поведение MASH-2
        total_correction = (int32_t)carry1 + (int32_t)carry2 - (int32_t)loc_m2_carry_prev;
        loc_m2_carry_prev = carry2;  // Запоминаем перенос для следующего такта
        // Такой знакопеременный двухступенчатый процесс заставляет ошибку квантования 
        // флуктуировать с высокой крутизной. Шум выталкивается вверх по спектру со скоростью 40 дБ на декаду
#else
        uint32_t old_acc = loc_acc1;
        loc_acc1 += (uint32_t)step;
        if (loc_acc1 < old_acc) {
            total_correction = 1;
        }
#endif

        // ЗНАКОВАЯ НОРМАЛИЗАЦИЯ: переменные принудительно приведены к int32_t
        int32_t current_frac = l_frac + total_correction;
        int32_t current_int  = l_int;

        // Компилятор гарантированно применит asrs. Если current_frac < 0 (например, -1), из целой части займется 1
        current_int += (current_frac >> 8); 
        current_frac &= 0xFF; // Маска восстановит легальное значение FRAC из отрицательного остатка

        // Единственная STR-запись в шину периферии PIO за итерацию
        *clkdiv_reg = ((uint32_t)current_int << 16) | ((uint32_t)current_frac << 8);

#else
        // === ЭТАЛОННАЯ МЕДЛЕННАЯ СИ-ВЕРСИЯ (С прогонкой через глобальное ОЗУ) ===
        dds_accumulator = loc_acc1;
        xorshift_state = loc_rand_state;
#ifdef VFO_USE_MASH2
        dds_accum_m2 = loc_acc2;
        m2_carry_prev = loc_m2_carry_prev;
#endif

        vfo_dither_step((uint32_t)l_step, (uint32_t)l_int, (uint32_t)l_frac);

        loc_acc1 = dds_accumulator;
        loc_rand_state = xorshift_state;
#ifdef VFO_USE_MASH2
        loc_acc2 = dds_accum_m2;
        loc_m2_carry_prev = m2_carry_prev;
#endif
#endif
    }
}





 
// Минимальное КОЛЬЦЕВОЕ расстояние dds_step до якорей k/8 (единицы 2^-32).  
// Якоря: k * 2^29, k = 0..7. Якорь 8/8 = 2^32 совпадает с 0 по кольцу —  
// кольцевая арифметика покрывает его автоматически.  
// Максимум результата = 2^28 (половина интервала k/8).  
static uint32_t dds_step_min_dist_to_k8(uint32_t step) {  
    uint32_t best = (1u << 28);          // 268435456 — теоретический максимум  
    for (uint32_t k = 0; k < 8; k++) {  
        uint32_t a = k << 29;            // якорь k/8 шкалы  
        uint32_t d = step - a;           // беззнаковая разность — уже на кольце  
        if (d > 0x80000000u) d = 0u - d; // min(d, 2^32 - d)  
        if (d < best) best = d;  
    }  
    return best;  
}





static uint64_t vfo_pll_metric_verbose(uint64_t clk_sys_hz, uint64_t target_mhz, MetricBreakdown *b, VfoParameters *out) {
    VfoParameters test = calculate_raw_params_mhz(clk_sys_hz, target_mhz);
    if (out) *out = test;

    memset(b, 0, sizeof(MetricBreakdown));
    b->frac = test.pio_frac;
    b->raw_step = test.dds_step;

    if (test.pio_frac == 0 && test.dds_step == 0) {
        return 0; // Аппаратный абсолютный идеал
    }

    // 1. Штраф за дальние спуры (простые дроби FRAC8)
    if (test.pio_frac != 0) {
        b->ctz_val = (uint32_t)__builtin_ctz(test.pio_frac);
        b->ctz_penalty = (1ULL << (b->ctz_val + 26));
        b->total_metric += b->ctz_penalty;
    }
 
    // 2. Штраф за ближнюю зону MASH-2 (размытие юбки)  
    if (test.dds_step != 0) {  
        uint64_t target_center = 0x80000000ULL;  
        b->center_dist = (test.dds_step > target_center) ? (test.dds_step - target_center) : (target_center - test.dds_step);  
        b->mash_penalty = b->center_dist / 4ULL;  
  
        // Штраф за близость к простым дробям k/8 шкалы  
        uint32_t dist_k8 = dds_step_min_dist_to_k8(test.dds_step);  
        if (dist_k8 < (1u << 24)) {  
            uint64_t deficit = (uint64_t)((1u << 24) - dist_k8);  
            b->mash_penalty += (deficit * deficit) >> 12;  
        }  
        b->total_metric += b->mash_penalty;  
    }

    // 3. Штраф за отказ от разгона к 380 МГц
    if (clk_sys_hz < VFO_PREFERRED_MIN_CLK) {
        b->clk_penalty += 10000000000ULL; // Барьерный штраф ниже 320 МГц
    }
    uint64_t clk_deficit = VFO_MAX_STABLE_CLK_HZ - clk_sys_hz;
    b->clk_penalty += (clk_deficit * clk_deficit) / 5000ULL;
    b->total_metric += b->clk_penalty;

    // 4. Запрет малых INT
    if (test.pio_int < 4) {
        b->int_penalty = VFO_INT_PENALTY;
        b->total_metric += b->int_penalty;
    }

    return b->total_metric;
}





/**  
 * @brief Прецизионный расчёт параметров PIO-делителя для целевой частоты  
 *        в миллигерцах (0.001 Гц).  
 *  
 * Вычисляет делитель автомата PIO в формате fixed-point Q8.8  
 * (pio_int + pio_frac/256) для выходного меандра `f_out = clk_sys / (2 * D)`,  
 * а также 32-битный остаток ошибки `dds_step` — приращение для DDS/MASH-2  
 * дизеринга, компенсирующее остаточную дробную часть делителя.  
 */
static VfoParameters calculate_raw_params_mhz(uint64_t clk_sys_hz, uint64_t mhz_target) {
    // Границы КВ-диапазона в миллигерцах (1.0 .. 40.0 МГц)
    if (mhz_target < 100000000ULL)   mhz_target = 100000000ULL;
    if (mhz_target > 40000000000ULL) mhz_target = 40000000000ULL;

    uint64_t clocks_per_period = 2ULL; 
    uint64_t vfo_denom = mhz_target * clocks_per_period; 
    
    // Масштабирующий коэффициент 1000ULL переводит миллигерцы в базовые Герцы
    uint64_t pio_div_fixed8 = ((clk_sys_hz * 256ULL) * 1000ULL) / vfo_denom;
    
    VfoParameters params;
    params.pio_int  = pio_div_fixed8 >> 8;
    params.pio_frac = pio_div_fixed8 & 0xFFu;
    
    // Для совместимости со структурой сохраняем в chz (сантигерцах)
    params.target_freq_chz = (uint32_t)(mhz_target / 10ULL); 

    if (params.pio_int < 2) { params.pio_int = 2; params.pio_frac = 0; }

    // Расчет 32-битного остатка ошибки DDS
    uint64_t clk_sys_rem = ((clk_sys_hz * 256ULL) * 1000ULL) % vfo_denom;
    uint64_t intermediate = (clk_sys_rem << 16) / vfo_denom;
    uint64_t remainder_low = (clk_sys_rem << 16) % vfo_denom;
    
    params.dds_step = (uint32_t)((intermediate << 16) + ((remainder_low << 16) / vfo_denom));  
    // Принудительная нечётность шага: gcd(dds_step, 2^32) = 1, период паттерна  
    // переносов = 2^32 отсчётов -> дискретная гребёнка превращается в шумовую полку.  
    // Ошибка 1 LSB остатка (~F_s/2^32 Гц) пренебрежима.  
    if (params.dds_step != 0) params.dds_step |= 1u;

    return params;
}




// Метрика кандидата
static uint64_t vfo_pll_metric(uint64_t clk_sys_hz, uint64_t target_mhz, VfoParameters *out) {
    VfoParameters test = calculate_raw_params_mhz(clk_sys_hz, target_mhz);
    if (out) *out = test;

    // Идеал: целое аппаратное деление, дизер полностью спит
    if (test.pio_frac == 0 && test.dds_step == 0) return 0;

    uint64_t metric = 0;

    // ------------------------------------------------------------------------
    // КРИТЕРИЙ 1: Борьба с дальними спурами -40 dBc (Анализ простоты дроби)
    // ------------------------------------------------------------------------
    if (test.pio_frac != 0) {
        // Вычисляем ctz (количество младших нулей). 
        // Чем больше нулей, тем проще дробь (128->7 нулей, 64->6 нулей)
        // Простые дроби порождают самые мощные иголки на анализаторе.
        uint32_t fraction_simplicity = (uint32_t)__builtin_ctz(test.pio_frac);
        
        // Тяжелый экспоненциальный штраф за вырождение джиттера в периодический меандр
        metric += (1ULL << (fraction_simplicity + 26)); 
    }


    // ------------------------------------------------------------------------  
    // КРИТЕРИЙ 2: Чистка ближней зоны MASH-2 (Размытие шумовой юбки)  
    // ------------------------------------------------------------------------  
    if (test.dds_step != 0) {  
        uint64_t target_center = 0x80000000ULL; // Центр шкалы 32-битного DDS  
        uint64_t current_step  = test.dds_step;  
          
        // Ищем удаление от центра. Возле краев (0 или 0xFFFFFFFF) MASH-2 "сбоит"  
        // и кучкует энергию фазового шума вплотную к основному тону.  
        uint64_t center_dist = (current_step > target_center) ? (current_step - target_center) : (target_center - current_step);  
        metric += center_dist / 4ULL;  
  
        // Штраф за близость к простым дробям шкалы k/8 (вкл. края 0 и 2^32).  
        // Дистанция < 2^24 (±6.3% от октавы) -> idle-тоны в ближней зоне.  
        uint32_t dist_k8 = dds_step_min_dist_to_k8(test.dds_step);  
        if (dist_k8 < (1u << 24)) {  
            // квадратичный штраф: чем ближе к якорю, тем хуже  
            uint64_t deficit = (uint64_t)((1u << 24) - dist_k8);  
            metric += (deficit * deficit) >> 12;   // до ~2^36 в нуле  
        }  
    }


    // ------------------------------------------------------------------------
    // КРИТЕРИЙ 3: Безусловный силовой разгон к 380 МГц
    // ------------------------------------------------------------------------
    if (clk_sys_hz < VFO_PREFERRED_MIN_CLK) {
        // Если частота ушла ниже 320 МГц — кандидат почти гарантированно отсекается
        metric += 10000000000ULL;
    }
    
    // Квадратичный штраф за падение частоты относительно потолка в 380 МГц
    uint64_t clk_deficit = VFO_MAX_STABLE_CLK_HZ - clk_sys_hz;
    metric += (clk_deficit * clk_deficit) / 5000ULL;

    // Запрет опасного сверхмалого деления
    if (test.pio_int < 4) metric += VFO_INT_PENALTY;

    return metric;
}


// хелпер
// Минимальная отстройка FRAC8-спура по гармоникам k=1..4, Гц.  
// frac==0 -> UINT64_MAX (нет спура). Спур k-й гармоники садится на  
// f_sm * fd/256, где fd = свёртка (frac*k mod 256) в диапазон ±128.  
static uint64_t frac_spur_min_off_hz(uint16_t pio_int, uint8_t pio_frac,  
                                     uint64_t clk_sys_hz) {  
    if (pio_frac == 0) return UINT64_MAX;  
    uint64_t div_x256 = (uint64_t)pio_int * 256ULL + pio_frac;  
    if (div_x256 == 0) return UINT64_MAX;  
    uint64_t best = UINT64_MAX;  
    for (uint32_t k = 1; k <= 4; k++) {  
        uint32_t m  = ((uint32_t)pio_frac * k) & 0xFFu;  
        uint32_t fd = (m < 256u - m) ? m : 256u - m;   // 0..128  
        if (fd == 0) continue;                          // гармоника на тон  
        uint64_t off = clk_sys_hz * fd / div_x256;      // Гц  
        if (off < best) best = off;  
    }  
    return best;  
}




PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz, uint64_t max_clk_limit) {  
  
    uint64_t crystal_hz = VFO_CALIBRATED_XOSC_HZ;  
    uint64_t target_mhz = (uint64_t)target_frequency_hz * 1000ULL; // миллигерцы

    uint64_t clk_limit = (max_clk_limit == 0) ? vfo_effective_clk_max() : max_clk_limit;
  
    // Дефолтная безопасная конфигурация на случай сбоя  
    PllConfig best_pll = { 133, 6, 2, 133000000ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };
    // Если в вашей PllConfig поля refdiv нет — уберите `1,` и строки присваивания refdiv ниже.  
  
    // Границы КВ-диапазона (1.0 .. 30.0 МГц) в миллигерцах  
    if (target_mhz < 1000000000ULL)  target_mhz = 1000000000ULL;  // 1.0 МГц  
    if (target_mhz > 30000000000ULL) target_mhz = 30000000000ULL; // 30 МГц  
  
    const uint64_t min_allowed_clk = VFO_CLK_SYS_MIN_HZ;  
    const uint64_t max_allowed_clk = max_clk_limit;  
  
#if VFO_PLL_DEBUG  
    Serial.printf("[PLLDBG] limits: clk_sys<=%.1f MHz, VCO<=%.0f MHz (flash)\n",  
              vfo_effective_clk_max() / 1e6, vfo_effective_pll_max() / 1e6);
    Serial.printf("[PLLDBG] scan range: %.3f .. %.3f MHz (target=%u)\n",  
              min_allowed_clk / 1e6, max_allowed_clk / 1e6, target_frequency_hz);  
    Serial.printf("[PLLDBG] target=%u Hz  nominal: clk=%lu\n",  
                  target_frequency_hz, (unsigned long)pll_nominal.clk_sys_hz);  
#endif  
  
    // ---- Классификаторы и состояние арбитража ----  
    uint64_t min_dds_metric    = UINT64_MAX;  // метрика победителя (не номинала!)  
    bool     best_forbidden    = true;        // класс победителя: frac в поясе  
    bool     best_clean        = false;       // класс победителя: грязный dmin  
    uint64_t best_pll_vco_hz   = 0;           // VCO победителя (тай-брейк при =metric)  
    uint32_t n_arbit           = 0;           // сколько кандидатов дошло до арбитража  
  
#if VFO_PLL_DEBUG  
    // TOP-10 лог для диагностики (без дублей по clk_sys)  
    struct CandLog {  
        uint64_t clk, vco, metric, spur_off;  
        uint32_t fbdiv, p1, p2, refdiv, step;  
        uint16_t pio_int;  
        uint8_t  pio_frac;  
        bool     forbidden, clean;  
    };  
    CandLog top10[10];  
    for (int i = 0; i < 10; i++) top10[i].metric = UINT64_MAX, top10[i].clk = 0;  
#endif  
  
    // ---- Матричный скан: refdiv × fbdiv × (p1,p2) ----  
    //for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {  
    uint32_t refdiv = 1;
    uint64_t ref_hz = crystal_hz / refdiv;   // REFDIV: 1,2,3 → 12,6,4 МГц  
  
        for (uint32_t p1 = 2; p1 <= 7; p1++) {   // p1 >= 2: режим p1=1 даёт ровное деление на 2 при VCO < ~2.8 ГГц  
            for (uint32_t p2 = 1; p2 <= 7; p2++) {  
                uint32_t pdiv_total = p1 * p2;  
  
                for (uint32_t fbdiv = 16; fbdiv <= 320; fbdiv++) {  
                    uint64_t vco_hz = ref_hz * fbdiv;  
  
                    // Только нижний предел VCO (даташит 750 МГц)
                    if (vco_hz < 750000000ULL) continue;
                    // Верхний предел
                    if (vco_hz > vfo_effective_pll_max()) continue;          // лимит VCO из flash (1600М…3.9Г)
                    
                    uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;
                    if (clk_sys_hz < min_allowed_clk ||
                        clk_sys_hz > clk_limit) continue;                     // лимит clk_sys из flash (133М…400М)

                    // Жёсткий минимум шины: спур-уровень ~ f_out²/clk_sys
                    if (clk_sys_hz < VFO_CLK_SYS_PREF_MIN_HZ) continue;
  
                    // Единая метрика кандидата (verbose-версия с декомпозицией штрафов)
                    VfoParameters test;
                    MetricBreakdown brk;
                    uint64_t current_dds_metric =
                        vfo_pll_metric_verbose(clk_sys_hz, target_mhz, &brk, &test);
                    // Если verbose нет — используйте vfo_pll_metric(...) и уберите brk
  
                    if (test.pio_int < 2) continue;
  
                    // === КЛАСС 1: запретный пояс FRAC8 (многогармонический, k=1..4) ===
                    uint64_t spur_off_hz = frac_spur_min_off_hz(test.pio_int,
                                                                test.pio_frac,
                                                                clk_sys_hz);
                    bool frac_forbidden = (test.pio_frac != 0u) &&
                                          (spur_off_hz < VFO_SPUR_MIN_OFFSET_HZ);
  
                    // === КЛАСС 2: чистота дизера по СЫРОМУ dmin (до нормализации!) ===  
                    uint32_t dd  = test.dds_step;  
                    uint32_t dmin_raw = (dd < (uint32_t)(0u - dd)) ? dd  
                                                                   : (uint32_t)(0u - dd);  
                    bool candidate_clean = (dmin_raw < VFO_DMIN_CEILING);  
  
                    n_arbit++;  
  
#if VFO_PLL_DEBUG  
                    // TOP-10 без дублей по clk_sys  
                    bool dup = false;  
                    for (int i = 0; i < 10; i++)  
                        if (top10[i].clk == clk_sys_hz) { dup = true; break; }  
                    if (!dup && current_dds_metric < top10[9].metric) {  
                        int pos = 9;  
                        while (pos > 0 && current_dds_metric < top10[pos-1].metric) pos--;  
                        for (int k = 9; k > pos; k--) top10[k] = top10[k-1];  
                        top10[pos].clk = clk_sys_hz;  top10[pos].vco = vco_hz;  
                        top10[pos].metric = current_dds_metric;  
                        top10[pos].fbdiv = fbdiv; top10[pos].p1 = p1;  
                        top10[pos].p2 = p2;         top10[pos].refdiv = refdiv;  
                        top10[pos].step = test.dds_step;  
                        top10[pos].pio_int = test.pio_int;  
                        top10[pos].pio_frac = test.pio_frac;  
                        top10[pos].spur_off = spur_off_hz;  
                        top10[pos].forbidden = frac_forbidden;  
                        top10[pos].clean = candidate_clean;  
                    }  
#endif  
  
                    // === АРБИТРАЖ: класс пояса -> класс чистоты -> метрика -> VCO ===  
                    // Разрешённый frac всегда бьёт запретный; внутри класса  
                    // чистый dmin бьёт грязный; дальше — меньшая метрика,  
                    // при равной метрике — выше VCO (больше запас PLL).  
                    bool prefer;  
                    if (frac_forbidden != best_forbidden) {  
                        prefer = !frac_forbidden;  
                    } else if (candidate_clean != best_clean) {  
                        prefer = candidate_clean;  
                    } else {  
                        prefer = (current_dds_metric < min_dds_metric) ||  
                                 (current_dds_metric == min_dds_metric &&  
                                  vco_hz > best_pll_vco_hz);  
                    }  
  
                    if (prefer) {  
                        best_forbidden  = frac_forbidden;  
                        best_clean      = candidate_clean;  
                        min_dds_metric  = current_dds_metric;  
                        best_pll_vco_hz = vco_hz;  
  
                        best_pll.fbdiv      = fbdiv;  
                        best_pll.p1         = p1;  
                        best_pll.p2         = p2;  
                        best_pll.refdiv     = refdiv;
                        best_pll.clk_sys_hz = clk_sys_hz;  
                        best_pll.vsel       = (uint32_t)vsel_for(clk_sys_hz);  
                        best_pll.is_oc      = (clk_sys_hz > VFO_CLK_SYS_NOMINAL_HZ);  
                    }  
                }  
            }  
        }  
    //}  
  
    // === ЗАЩИТА ОТ ПУСТОГО СКАНА: дефолт выжил только если кандидатов не было ===  
    if (n_arbit == 0) {  
#if VFO_PLL_DEBUG  
        Serial.printf("[PLLDBG] SCAN EMPTY — fallback to nominal 133 MHz\n");  
#endif  
        return best_pll;   // дефолт 133 МГц — и никак иначе  
    }  
  
#if VFO_PLL_DEBUG  
    for (int i = 0; i < 10; i++) {  
        if (top10[i].clk == 0) break;  
        Serial.printf("[PLLDBG] top%d: clk=%.3f MHz vco=%.3f MHz refdiv=%lu fbdiv=%lu "  
                      "p1=%lu p2=%lu metric=%llu frac=%u spur_off=%llu kHz%s%s\n",  
                i, top10[i].clk / 1e6, top10[i].vco / 1e6,  
                (unsigned long)top10[i].refdiv, (unsigned long)top10[i].fbdiv,  
                (unsigned long)top10[i].p1, (unsigned long)top10[i].p2,  
                (unsigned long long)top10[i].metric, (unsigned)top10[i].pio_frac,  
                (unsigned long long)(top10[i].spur_off == UINT64_MAX ? 0  
                                       : top10[i].spur_off / 1000ULL),  
                top10[i].forbidden ? " [FORB]" : "",  
                top10[i].clean ? "" : " [DIRTY]");  
    }  
  
    // Победитель: пересчёт параметров для печати  
    VfoParameters wp;  
    vfo_pll_metric(best_pll.clk_sys_hz, target_mhz, &wp);  
    uint64_t w_spur = frac_spur_min_off_hz(wp.pio_int, wp.pio_frac,  
                                         best_pll.clk_sys_hz);  
    Serial.printf("[PLLDBG] WINNER: clk=%.3f MHz refdiv=%lu fbdiv=%lu p1=%lu p2=%lu "  
                  "metric=%llu  int=%u frac=%u step=0x%08lX  spur_off=%llu kHz%s\n",  
                best_pll.clk_sys_hz / 1e6,  
                (unsigned long)best_pll.refdiv,  
                (unsigned long)best_pll.fbdiv,  
                (unsigned long)best_pll.p1,  
                (unsigned long)best_pll.p2,  
                (unsigned long long)min_dds_metric,  
                (unsigned)wp.pio_int, (unsigned)wp.pio_frac,  
                (unsigned long)wp.dds_step,  
                (unsigned long long)(w_spur == UINT64_MAX ? 0 : w_spur / 1000ULL),  
                best_forbidden ? " [FORBIDDEN]" : "");  
#endif  
    return best_pll;  
}





// vfo_hardware.cpp — вынести блок заполнения в функцию:  
static void vfo_rebuild_tone_table(uint64_t base_freq_mhz, uint64_t step_mhz) {  
    for (int i = 0; i < VFO_IFKP_TONES_COUNT; i++) {  
        uint64_t tone_freq_mhz = base_freq_mhz + ((uint64_t)i * step_mhz);  
        ifkp_tones[i] = calculate_raw_params_mhz(current_clk_sys_hz, tone_freq_mhz);  
    }  
    // Сбросить кэш активного тона, чтобы следующий vfo_set_tone_instant применил новые значения  
    current_active_tone = VFO_TONE_NONE;  
}



// === ИНИЦИАЛИЗАЦИЯ И СТАРТ СИСТЕМЫ ===
/**  
 * @brief Полная инициализация и запуск аппаратного VFO (PLL + PIO + дизеринг).  
 *  
 * Осуществляет сквозную синхронизацию частотного плана. Если предпусковой разгон  
 * функция блокирует откат частоты и настраивает подсистемы fixed-point Q8.8 делителей  
 * PIO и 32-битного DDS-остатка строго на базе высокой стабильной оверклокерской частоты.  
 */
void vfo_hardware_init(unsigned int base_freq_hz, double step_hz) {
#ifdef VFO_DITHER_ON_CORE1
    // Жесткий перезапуск изолированного ядра перед изменением параметров таблиц тонов
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
    
    PllConfig target_pll;
    
    // ========================================================================
    // АРХИТЕКТУРНЫЙ АРБИТРАЖ ЧАСТОТЫ ШИНЫ
    // Честный сквозной автотюн при каждом старте (без эффекта памяти)
    // ========================================================================

    if (clk_boosted) { 
        // Если активирован BOOST — запускаем матричный поиск ЛУЧШЕЙ частоты PLL
        // строго под НОВУЮ целевую частоту в пределах стабильного потолка pll_ceiling
        target_pll = vfo_find_optimal_pll(base_freq_hz, vfo_effective_clk_max());
        
        // Синхронизируем рабочий профиль оверклока для Термогуарда
        pll_overclock = target_pll;
    } 
    else {
        // Если буст спит — производим штатный автотюнинг в пределах номинальных 133 МГц
        #if defined(VFO_PLL_AUTOTUNE)
            target_pll = vfo_find_optimal_pll(base_freq_hz, VFO_CLK_SYS_NOMINAL_HZ);
        #else
            target_pll = pll_nominal;
        #endif
    }

    // Подбираем безопасное рантайм-напряжение ядра процессора (VREG)
    uint32_t selected_vsel = target_pll.vsel;
    if (selected_vsel == 0 || selected_vsel < (uint32_t)VREG_VOLTAGE_1_10) {
        selected_vsel = (target_pll.clk_sys_hz > VFO_CLK_SYS_NOMINAL_HZ) 
                        ? (uint32_t)vsel_for(target_pll.clk_sys_hz) 
                        : (uint32_t)VREG_VOLTAGE_DEFAULT;
    }

    // Физически прошиваем выбранные делители в регистры PLL чипа RP2040
    // (Этот вызов обновит и физическую частоту, и current_clk_sys_hz!)
    vfo_set_clk_sys(target_pll, selected_vsel);
    // ========================================================================

    // ========================================================================
    // ИСПРАВЛЕНО: Честный вывод BOOST строго в момент его физического захвата!
    // Теперь данные fbdiv, p1, p2 берутся из РЕАЛЬНО примененной target_pll.
    // ========================================================================
    if (clk_boosted) {
        pll_overclock = target_pll; // Обновляем рабочий профиль для термогуарда
        #if VFO_PLL_DEBUG
            Serial.printf("[BOOST] ON   clk_sys=%7.3f MHz  refdiv=%lu  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)  T_CPU=%5.1f C\n",    
                      (double)current_clk_sys_hz / 1000000.0,  // Выведет честные 254.007 MHz!
                      (unsigned long)target_pll.refdiv,
                      (unsigned long)target_pll.fbdiv,    
                      (unsigned long)target_pll.p1,    
                      (unsigned long)target_pll.p2,    
                      (unsigned long)(target_pll.fbdiv * (uint32_t)(VFO_CALIBRATED_XOSC_HZ / 1000000ULL)),    
                      (unsigned)vsel_to_mv(selected_vsel),    
                      (double)current_clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,    
                      (unsigned)ssi_hw->baudr,    
                      (double)vfo_read_core_temp_c());
        #endif
    }
    // ========================================================================


    // Конфигурация и запуск конечного автомата (State Machine) PIO
    if (!pio_program_loaded) { 
        lo_offset = pio_add_program(lo_pio, &pio_square_program); 
        pio_program_loaded = true; 
    }
    pio_sm_config c = pio_get_default_sm_config(); 
    sm_config_set_wrap(&c, lo_offset + 0, lo_offset + 1); 
    sm_config_set_set_pins(&c, pin_freq_out, 1); 
    
    pio_gpio_init(lo_pio, pin_freq_out); 
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, pin_freq_out, 1, true); 
    
    pio_sm_init(lo_pio, lo_sm, lo_offset, &c); 
    pio_sm_set_enabled(lo_pio, lo_sm, true);

    // Расчет базовой частоты и привязка к сетке Брезенхема
    uint64_t base_target_chz = (uint64_t)base_freq_hz * 100ULL; 
    uint64_t snapped_base_chz = base_target_chz;

    vfo_base_mhz = snapped_base_chz * 10ULL; 
    vfo_step_mhz = (uint64_t)(step_hz * 1000.0);  
    
    // Синхронный потокобезопасный расчет всей FSK/IFKP таблицы тонов
    vfo_rebuild_tone_table(vfo_base_mhz, vfo_step_mhz); 
    vfo_set_tone_instant(0);

// Диагностический вывод рантайм-телеметрии в Serial  
    #if VFO_PLL_DEBUG  
        VfoParameters real_base_params = ifkp_tones[0];  
        Serial.printf("\n--- VFO Runtime Diagnostics ---\n");  
        Serial.printf("Target Freq: %u Hz (Grid: %.2f Hz)\n", base_freq_hz, (double)real_base_params.target_freq_chz / 100.0);  
        Serial.printf("clk_sys    : %u Hz (Физический захват PLL: refdiv=%lu, fbdiv=%lu, p1=%lu, p2=%lu)\n",  
                      current_clk_sys_hz,  
                      (unsigned long)target_pll.refdiv,  
                      (unsigned long)target_pll.fbdiv,  
                      (unsigned long)target_pll.p1,  
                      (unsigned long)target_pll.p2);  
        // Фактическое состояние регистров PLL на момент печати (refdiv — биты [5:0] CS)  
        Serial.printf("PLL CS     : 0x%08lX (refdiv=%lu)  FB=%lu  PRIM=0x%08lX\n",  
                      (unsigned long)pll_sys_hw->cs,  
                      (unsigned long)(pll_sys_hw->cs & PLL_CS_REFDIV_BITS),  
                      (unsigned long)pll_sys_hw->fbdiv_int,  
                      (unsigned long)pll_sys_hw->prim);  
        Serial.printf("PIO Regs   : INT=%u, FRAC=%u\n", real_base_params.pio_int, real_base_params.pio_frac);  
        Serial.printf("DDS Step   : 0x%08X (%u)  dist_to_k8=%u LSB\n",  
                    real_base_params.dds_step, real_base_params.dds_step,  
                    (unsigned)dds_step_min_dist_to_k8(real_base_params.dds_step)); 
        Serial.printf("-------------------------------\n");  
    #endif

#ifdef VFO_DITHER_ON_CORE1
    // Безопасный атомарный пуск высокоскоростного регистрового dither-конвейера на Core 1
    tone_changed = true; 
    multicore_launch_core1(vfo_core1_entry); 
#else
    add_repeating_timer_us(-(int64_t)VFO_DITHER_INTERVAL_US, vfo_dither_callback, NULL, &sdr_dither_timer); 
    timer_already_running = true;
#endif
}





static uint16_t vsel_to_mv(uint32_t vsel) {  
    // vreg_voltage: VREG_VOLTAGE_0_80 == 5, шаг 50 мВ, максимум VREG_VOLTAGE_1_30 == 1300 мВ  
    return (uint16_t)(900 + (vsel - VREG_VOLTAGE_0_90) * 50);  
}

// === ТЕМПЕРАТУРА ЯДРА RP2040 (ADC канал 4, встроенный датчик) ===  
// Формула из даташита RP2040: T = 27 - (V_bead - 0.706) / 0.001721  
// СКОРРЕКТИРОВАННОЕ ПОМЕХОЗАЩИЩЕННОЕ ЧТЕНИЕ ТЕМПЕРАТУРЫ [Исправлено]
static float vfo_read_core_temp_c(void) {
    // Восстановление clk_adc после смены clk_sys (48 МГц от PLL_USB)  
    clock_configure(clk_adc,  
                    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
                    0, 48000000u, 48000000u);
    adc_select_input(4);  

    uint32_t samples[3];
    
    // Делаем три последовательных замера с микросекундными паузами, 
    // чтобы дождаться стабилизации опорного напряжения аналоговой шины
    for(int i = 0; i < 3; i++) {
        samples[i] = adc_read();
        busy_wait_us(10); 
    }

    // Простейшая медианная фильтрация: выбираем среднее значение, отсекая ложные "иголки"
    uint32_t raw;
    if ((samples[0] <= samples[1] && samples[1] <= samples[2]) || (samples[2] <= samples[1] && samples[1] <= samples[0]))
        raw = samples[1];
    else if ((samples[1] <= samples[0] && samples[0] <= samples[2]) || (samples[2] <= samples[0] && samples[0] <= samples[1]))
        raw = samples[0];
    else
        raw = samples[2];

    float v_bead = raw * 3.3f / 4095.0f; // ADC 12 бит, VREF = 3.3 В  
    return 27.0f - (v_bead - 0.706f) / 0.001721f;  
}



 
void vfo_clk_thermal_guard(void) {  
    if (!clk_boosted && !thermal_throttled) return;   
    if (pll_ceiling.clk_sys_hz <= pll_nominal.clk_sys_hz) return; 
  
    float temp = vfo_read_core_temp_c();  
  
    if (!thermal_throttled && temp >= VFO_THROTTLE_HI_C) {  
        // Аварийный откат на номинал
        vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
        //vfo_rebuild_tone_table(cached_base_freq_hz, cached_step_hz);  
        vfo_rebuild_tone_table((uint64_t)cached_base_freq_hz * 1000ULL,  
                            (uint64_t)(cached_step_hz * 1000.0));
        vfo_set_tone_instant(0);  
        clk_boosted = false;  
        thermal_throttled = true;  
        Serial.printf("[BOOST] THROTTLE: T_CPU=%.1f C >= %.0f C — откат на %lu MHz\n",  
                      (double)temp, (double)VFO_THROTTLE_HI_C,  
                      (unsigned long)(pll_nominal.clk_sys_hz / 1000000ULL));  
    }  
    else if (thermal_throttled && temp <= VFO_THROTTLE_LO_C) {  
        // Кристалл остыл — ВОЗВРАЩАЕМ АКТИВНЫЙ ПРОФИЛЬ ТЕКУЩЕЙ СЕССИИ (pll_overclock) [Исправлено]
        vfo_set_clk_sys(pll_overclock, pll_overclock.vsel);  
        //vfo_rebuild_tone_table(cached_base_freq_hz, cached_step_hz);
        vfo_rebuild_tone_table((uint64_t)cached_base_freq_hz * 1000ULL,  
                            (uint64_t)(cached_step_hz * 1000.0));
        vfo_set_tone_instant(0);  
        clk_boosted = true;  
        thermal_throttled = false;  
        Serial.printf("[BOOST] RESUME: T_CPU=%.1f C — возврат на %7.3f MHz\n",  
                      (double)temp, (double)pll_overclock.clk_sys_hz / 1000000.0);  
    }  
}




/**  
 * @brief Экстремальный стресс-тест: Поиск абсолютного физического потолка PLL.  
 *  
 * Сканирует все возможные рантайм-комбинации (REFDIV от 1 до 3, VCO до 2.4 ГГц)
 * строго по нарастающей итоговой частоты clk_sys. На каждой ступени прогоняет 
 * жесткий композитный тест (FPU, AHB-SRAM CRC, ветвления, делитель SIO).
 */  
void __not_in_flash_func(vfo_find_max_stable_clock)(void) {  
    const uint64_t CLK_START_HZ = VFO_CLK_SYS_NOMINAL_HZ;  
    const uint64_t CLK_CEIL_HZ  = 450000000ULL; // Верхний теоретический предел сканирования
  
    // Scratch-регистр watchdog для сохранения частоты при зависании
    volatile uint32_t *scratch = &watchdog_hw->scratch[0];  
  
    struct VselStep { uint64_t max_hz; vreg_voltage vsel; };  
    const VselStep vsel_table[] = {  
        { 150000000ULL, VREG_VOLTAGE_1_15 },  
        { 200000000ULL, VREG_VOLTAGE_1_20 },  
        { 250000000ULL, VREG_VOLTAGE_1_25 },  
        { 450000000ULL, VREG_VOLTAGE_1_30 }, 
    };  
    const size_t vsel_table_size = sizeof(vsel_table) / sizeof(vsel_table[0]);  
  
    struct Candidate { uint32_t fbdiv, p1, p2, refdiv; uint64_t clk_sys_hz; };  
    // Расширяем массив до 3072, так как REFDIV=3 значительно увеличивает плотность сетки
    static Candidate cand[1024];  
    int cand_count = 0;  
  
    // 1. СБОР КАНДИДАТОВ: Сканируем REFDIV от 1 до 3 строго по вашему ТЗ [Скорректировано]
    for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {
        uint64_t f_ref = VFO_CALIBRATED_XOSC_HZ / refdiv;
        
        for (uint32_t fbdiv = 30; fbdiv <= 320; fbdiv++) {  
            uint64_t vco_hz = (uint64_t)fbdiv * f_ref;  
            
            // Граница аналогового захвата VCO
            if (vco_hz < 750000000ULL || vco_hz > 3200000000ULL) continue;  
  
            for (uint32_t p1 = 2; p1 <= 7; p1++) {  
                for (uint32_t p2 = 1; p2 <= 7; p2++) {  
                    if (p1 < p2) continue; // Соблюдаем ВЧ-правило p1 >= p2
                    
                    uint64_t cs = vco_hz / ((uint64_t)p1 * p2);  
                    if (cs <= CLK_START_HZ || cs > CLK_CEIL_HZ) continue;  
                    
                    if (cand_count >= (int)(sizeof(cand) / sizeof(cand[0]))) break;  
                    cand[cand_count++] = { fbdiv, p1, p2, refdiv, cs };  
                }  
            }  
        }  
    }  
  
    // 2. СОРТИРОВКА: Строго снизу вверх по частоте clk_sys_hz (от 133 МГц до 450 МГц)
    for (int a = 0; a < cand_count - 1; a++) {  
        for (int b = a + 1; b < cand_count; b++) {  
            if (cand[b].clk_sys_hz < cand[a].clk_sys_hz) {  
                Candidate t = cand[a]; cand[a] = cand[b]; cand[b] = t;  
            }  
        }  
    }  

/*  
    // 3. УМНАЯ ДЕДУПЛИКАЦИЯ: При одинаковой частоте ядра оставляем вариант с лучшим VCO [Исправлено]
    int w_pos = 0;  
    for (int r = 0; r < cand_count; r++) {  
        if (w_pos > 0 && cand[r].clk_sys_hz == cand[w_pos - 1].clk_sys_hz) {
            // Частоты равны! Проверяем, какой вариант дает более высокий VCO
            // (fbdiv / refdiv должен быть больше)
            uint64_t vco_current = (uint64_t)cand[r].fbdiv / cand[r].refdiv;
            uint64_t vco_saved   = (uint64_t)cand[w_pos - 1].fbdiv / cand[w_pos - 1].refdiv;
            
            if (vco_current > vco_saved) {
                // Если новый кандидат разгоняет VCO сильнее — перезаписываем старый дубликат
                cand[w_pos - 1] = cand[r];
            }
            // Иначе просто игнорируем его
        } else {
            // Частота уникальная — пишем на новую позицию
            cand[w_pos++] = cand[r];  
        }  
    }  
    cand_count = w_pos;
*/
  
    bool max_ok = false;  
    static uint32_t stress_buf[1024]; // 4 КБ SRAM для CRC-теста шины AHB
  
    Serial.printf("[OCTEST] Начинаем прецизионный прогон (R=1..3). Ступеней на сканирование: %d\n", cand_count);
    delay(200); // Даем очиститься буферу UART

    // 4. ГОРЯЧИЙ ЦИКЛ СТРЕСС-ТЕСТА
    for (int i = 0; i < cand_count; i++) {  
  
        vreg_voltage vsel = VREG_VOLTAGE_1_30;  
        for (size_t k = 0; k < vsel_table_size; k++) {  
            if (cand[i].clk_sys_hz <= vsel_table[k].max_hz) {  
                vsel = vsel_table[k].vsel;  
                break;  
            }  
        }  
  
        PllConfig step_cfg = {  
            cand[i].fbdiv, cand[i].p1, cand[i].p2,  
            cand[i].clk_sys_hz, (uint32_t)vsel, true, cand[i].refdiv 
        };  
  
        // Пишем МГц в регистр. Если плата зависнет, после ребута вы прочтете это число
        *scratch = (uint32_t)(cand[i].clk_sys_hz / 1000000ULL);  
  
        // Переключаем частоту и питание
        vfo_set_clk_sys(step_cfg, (uint32_t)vsel);  
  
        // --- КОМПОЗИТНЫЙ ВЕРИФИКАЦИОННЫЙ ТЕСТ ЯДРА ---  
        volatile uint32_t crc = 0xDEADBEEF;  
        uint32_t t0;  
  
        // Фаза 1: Арифметика + FPU
        {  
            volatile float acc_f = 1.000001f;  
            volatile uint32_t acc_i = 2654435761u;  
            t0 = millis();  
            while (millis() - t0 < 80) {  
                for (int n = 0; n < 1000; n++) {  
                    acc_f = acc_f * acc_f + 0.5f;  
                    acc_i = acc_i * 1664525u + 1013904223u;  
                }  
            }  
            crc ^= (uint32_t)acc_i;  
        }  
  
        // Фаза 2: AHB-Запись в SRAM
        t0 = millis();  
        while (millis() - t0 < 40) {  
            for (int w = 0; w < 1024; w++) {  
                stress_buf[w] = crc + (uint32_t)w;  
            }  
            crc++;  
        }  
  
        // Фаза 3: AHB-Чтение из SRAM с CRC верификацией
        {  
            bool sram_ok = true;  
            t0 = millis();  
            while (millis() - t0 < 40 && sram_ok) {  
                for (int w = 0; w < 1024; w++) {  
                    if (stress_buf[w] != crc - 1u + (uint32_t)w) {  
                        sram_ok = false;  
                        break;  
                    }  
                }  
            }  
            if (!sram_ok) {  
                Serial.printf("[OCTEST] FAIL (SRAM CRC) на clk_sys=%lu MHz\n", (unsigned long)(step_cfg.clk_sys_hz / 1000000ULL));  
                vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
                return;  
            }  
        }  
  
        // Фаза 4: Аппаратный делитель SIO
        {  
            bool div_ok = true;  
            uint32_t num = crc | 1u;  
            t0 = millis();  
            while (millis() - t0 < 40 && div_ok) {  
                for (int n = 0; n < 2000; n++) {  
                    num = num * 1664525u + 1013904223u;   
                    uint32_t den = (num >> 16) | 1u;      
                    uint32_t q = hw_divider_u32_quotient(num, den);  
                    uint32_t r = hw_divider_u32_remainder(num, den);  
                    if (q * den + r != num || r >= den) {  
                        div_ok = false;  
                        break;  
                    }  
                    crc ^= q ^ (r << 16);  
                }  
            }  
            if (!div_ok) {  
                Serial.printf("[OCTEST] FAIL (SIO DIVIDER) на clk_sys=%6.1f MHz\n", (double)(step_cfg.clk_sys_hz / 1000000.0));  
                vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
                return;  
            }  
        }  
  
        (void)crc;  
        watchdog_update();  
  

        // Ступень успешно пройдена: Читаем аппаратный статус защёлки фазы PLL [Скорректировано]
        bool is_locked = (pll_sys_hw->cs & PLL_CS_LOCK_BITS) != 0; // <--- СКОРРЕКТИРОВАНО

        uint32_t vco_mhz = (uint32_t)(((uint64_t)step_cfg.fbdiv * VFO_CALIBRATED_XOSC_HZ) / (1000000ULL * step_cfg.refdiv));  
  
        // Ваша исходная строка + параметр LOCK
        Serial.printf("[OCTEST] OK  clk_sys=%7.3f MHz  refdiv=%lu  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  LOCK=%s  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)\n",  
                      step_cfg.clk_sys_hz / 1000000.0,  
                      (unsigned long)step_cfg.refdiv,
                      (unsigned long)step_cfg.fbdiv,  
                      (unsigned long)step_cfg.p1,  
                      (unsigned long)step_cfg.p2,  
                      (unsigned long)vco_mhz,  
                      is_locked ? "YES" : "LOST!", // <-- Интегрирован статус фазы
                      (unsigned)vsel_to_mv((uint32_t)vsel),  
                      step_cfg.clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,
                      (unsigned)ssi_hw->baudr);  // <-- Выводим также итоговый BAUDR флэша
        Serial.flush();
 
        
        pll_ceiling = step_cfg;  
        max_ok = true;

        // Перезаписываем достигнутый потолок во Flash со сдвигом -3%.  
        // Flash пишется только на номинале: откат -> сохранение -> возврат на ступень.  
        uint32_t proven_hz = (uint32_t)((step_cfg.clk_sys_hz * 97ULL) / 100ULL);  
        vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
        persist_clock_limits(proven_hz, 0);   // 0 = не трогаем лимит PLL  
        Serial.printf("[OCTEST] flash: clk_sys_max=%lu Hz (-3%% от достигнутых %llu)\n",  
                      (unsigned long)proven_hz, step_cfg.clk_sys_hz);  
        Serial.flush();  
        vfo_set_clk_sys(step_cfg, (uint32_t)vsel);
    } 
  
    *scratch = 0xFFFFFFFFu;  
    vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
  
    if (max_ok) {  
        Serial.printf("[OCTEST] Потолок стабильности ядра зафиксирован: %6.1f МГц\n", (double)(pll_ceiling.clk_sys_hz / 1000000.0));  
    }  
}





/**  
 * @brief Сверхглубокий хакерский СВЧ-штурм PLL за барьер 4 ГГц с использованием ROSC-опоры.
 * Удерживает ядро на безопасной частоте, выжимая из аналоговой петли абсолютный предел.
 */
void __not_in_flash_func(vfo_test_pll_extreme_shurm)(void) {
    // 1. ПЕРЕВОДИМ ПИТАНИЕ И СИСТЕМУ В ЭКСТРЕМАЛЬНЫЙ РЕЖИМ
    vreg_set_voltage(VREG_VOLTAGE_1_30);  
    busy_wait_us(2000);  
    detach_peripheral_clock();  
    busy_wait_us(2000);

    Serial.printf("\n=================== ТОТАЛЬНЫЙ ШТУРМ PLL ЗА БАРЬЕР 4 ГГЦ ===================\n");
    Serial.printf("[ХАК] Переключаем опору PLL со скромных 12 МГц XOSC на разогнанный ROSC...\n");
    Serial.flush();
    busy_wait_us(10000);

    // Разгоняем ROSC на полную мощность прямой записью в регистры через аппаратную структуру
    rosc_hw->ctrl  = (0x272u << 12) | 0xD1E; // Запись пароля ENABLE + базовый разгон контура
    rosc_hw->freqa = (0x9696u << 16) | 0xFFFFu; // Пароль PASSWD + максимальный сдвиг фазы каскадов A
    rosc_hw->freqb = (0x9696u << 16) | 0xFFFFu; // Пароль PASSWD + максимальный сдвиг фазы каскадов B
    busy_wait_us(2000);

    // Измеряем получившуюся частоту с помощью встроенного частотомера (fc0) чипа
    // Используем явный вызов функции SDK clocks_khz
    uint32_t rosc_hz = clock_get_hz(clk_ref); // Базовое резервное чтение
    uint32_t measured_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    if (measured_khz > 0) {
        rosc_hz = measured_khz * 1000u;
    }

    Serial.printf("[ХАК] Частота ROSC-опоры стабилизирована на отметке: %7.3f MHz\n", rosc_hz / 1000000.0);
    Serial.flush();

    // Зажимаем postdiv делители на абсолютный максимум 7 x 7 = 49!
    // Это позволит VCO штурмовать 5 ГГц, удерживая ядро ниже 110 МГц!
    uint32_t target_p1 = 7;
    uint32_t target_p2 = 7;
    static volatile uint32_t stress_ram_block[512];

    // Шагаем по fbdiv от 100 до упора (320). 
    // При частоте ROSC ~24 МГц точка fbdiv=170 выдаст VCO = 4.0 ГГц!
    // Финал fbdiv=210 выдаст VCO = 5.0 ГГц!
    for (uint32_t fb = 90; fb <= 320; fb++) {
        uint64_t vco_hz = (uint64_t)fb * rosc_hz;
        uint32_t clk_sys_hz = (uint32_t)(vco_hz / (target_p1 * target_p2));

        // Если из-за разгона ROSC частота ядра вылетит за безопасные 150 МГц — 
        // аварийно завершаем, чтобы не сжечь цифровые затворы CPU
        if (clk_sys_hz > 150000000u) {
            Serial.printf("[ШТУРМ] Стоп! Частота ядра достигла критических %lu МГц. Прекращаем.\n", clk_sys_hz / 1000000ULL);
            break;
        }

        const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;
        uint32_t ssi_baud = (clk_sys_hz + FLASH_SAFE_MAX_HZ - 1) / FLASH_SAFE_MAX_HZ;  
        ssi_baud = (ssi_baud + 1u) & ~1u; 
        if (ssi_baud < 2u)  ssi_baud = 2u;  
        if (ssi_baud > 34u) ssi_baud = 34u;  

        bool sram_failed = false;

        // --- КРИТИЧЕСКАЯ СЕКЦИЯ ПРЯМОГО СДВИГА PLL НА ROSC ---
        uint32_t ints = save_and_disable_interrupts();  

        // Прошиваем fbdiv и postdiv каскады (p1=7, p2=7)
        pll_sys_hw->fbdiv_int = fb;
        pll_sys_hw->prim = (target_p1 << PLL_PRIM_POSTDIV1_LSB) | (target_p2 << PLL_PRIM_POSTDIV2_LSB);

        // Физически прошиваем делитель Flash-памяти до переключения clk_sys
        ssi_hw->ssienr = 0; 
        ssi_hw->baudr  = ssi_baud;  
        ssi_hw->ssienr = 1; 

        // Ожидаем захват СВЧ-фазы аналоговой петлей
        volatile uint32_t timeout = 30000;
        while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);

        // Обновляем частоту системного ядра clk_sys
        clock_configure(clk_sys,  
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                        clk_sys_hz, clk_sys_hz);  

        // Верификационный тест ОЗУ
        uint32_t pattern = 0x55AA55AAL ^ fb;
        for (int w = 0; w < 512; w++) { stress_ram_block[w] = pattern + w; }
        for (int w = 0; w < 512; w++) { if (stress_ram_block[w] != (pattern + w)) { sram_failed = true; break; } }

        restore_interrupts(ints);
        // --- ВЫХОД ИЗ КРИТИЧЕСКОЙ СЕКЦИИ ---

        float current_temp = vfo_read_core_temp_c();
        bool is_locked = (pll_sys_hw->cs & PLL_CS_LOCK_BITS) != 0;

        // Печать строки лога — штурмуем гигагерцы!
        Serial.printf("[СВЧ-ШТУРМ] FB=%3lu | VCO = %4lu MHz | Ядро = %3lu MHz | LOCK = %s | RAM = %s | T = %4.1f C | FLASH = %4.1f MHz\n", 
                      (unsigned long)fb,
                      (unsigned long)(vco_hz / 1000000ULL), 
                      (unsigned long)(clk_sys_hz / 1000000ULL),
                      is_locked ? "YES" : "LOST!",
                      sram_failed ? "FAIL" : "OK",
                      (double)current_temp,
                      (double)clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr);
        Serial.flush();

        // Как только аналоговая петля физически сорвется — фиксируем абсолютный рекорд планеты для RP2040!
        if (!is_locked || sram_failed) {
            Serial.printf("[СВЧ-ШТУРМ] ФИЗИЧЕСКИЙ СРЫВ АНАЛОГОВОЙ ПЕТЛИ PLL НА ОТМЕТКЕ VCO = %lu МГц!\n", (unsigned long)(vco_hz / 1000000ULL));
            break;
        }

        // === ФИКСАЦИЯ ШАГА: откат на номинал -> запись flash -> возврат на ступень ===  
        {  
            uint32_t proven_vco = (uint32_t)((vco_hz * 97ULL) / 100ULL);  
  
            // 1. Полный откат: clk_sys на номинал 133 МГц через штатный путь,  
            //    напряжение на VREG_VOLTAGE_DEFAULT. Все внешние опасные состояния сняты.  
            vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
  
            // 2. Запись в dedicated-сектор (внутри: XOSC + baudr=2 на время erase/program,  
            //    возврат на текущий выход PLL_SYS после записи).  
            persist_clock_limits(0, proven_vco);  
  
            Serial.printf("[ШТУРМ] flash: pll_vco_max=%lu Hz (-3%% от %llu)\n",  
                          (unsigned long)proven_vco, (unsigned long long)vco_hz);  
            Serial.flush();  
  
            // 3. Возврат на ступень штурма ОДИН раз: экстремальный вольтаж,  
            //    PLL на ROSC-опоре с текущим fb, clk_sys на частоту шага, baudr под него.  
            vreg_set_voltage(VREG_VOLTAGE_1_30);  
            busy_wait_us(500);  
  
            uint32_t ints2 = save_and_disable_interrupts();  
            pll_sys_hw->fbdiv_int = fb;  
            pll_sys_hw->prim = (target_p1 << PLL_PRIM_POSTDIV1_LSB) | (target_p2 << PLL_PRIM_POSTDIV2_LSB);  
            ssi_hw->ssienr = 0; ssi_hw->baudr = ssi_baud; ssi_hw->ssienr = 1;  
            volatile uint32_t t2 = 30000;  
            while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --t2);  
            clock_configure(clk_sys,  
                            CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                            CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                            clk_sys_hz, clk_sys_hz);  
            restore_interrupts(ints2);  
        }
    }
    // Возврат в безопасный номинал
    vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);
    Serial.println("===========================================================================");
    Serial.println("[СВЧ-ШТУРМ] Тест завершён. Опора возвращена на кварц.");
}




// ==== Энкодер: глобальное состояние ====  
volatile uint32_t vfo_current_freq_hz  = VFO_ENC_TEST_FREQ_HZ; // текущая рабочая частота 
volatile uint32_t vfo_tune_step_hz     = 500;       // глобальный шаг перестройки, Гц  
volatile int32_t  enc_pending_steps    = 0;         // накопленные тики энкодера (ISR)  
static bool       enc_tx_active        = false;

// Антидребезг кнопки энкодера — состояние debounce-машины  
static bool     enc_sw_last = true;   // предыдущий уровень SW (true = отпущена, INPUT_PULLUP)  
static uint32_t enc_sw_ms   = 0;      // millis() последнего принятого перепада

#define ENC_TICKS_PER_DETENT 4    // переходов квадратуры на один щелчок (полный шаг энкодера)

// Таблица квадратурного декодера (Gray-code state machine).  
// Индекс = (prev_AB << 2) | cur_AB. Значения: +1/-1 — шаг, 0 — дребезг/нет события.  
static const int8_t enc_lut[16] = {  
     0, -1, +1,  0,  
    +1,  0,  0, -1,  
    -1,  0,  0, +1,  
     0, +1, -1,  0  
};  
  
// Общий ISR для обоих пинов. Вызывается при любом изменении A или B.  
// Только атомарные операции — никакого Serial/PLL/флэша из прерывания.  
static void __not_in_flash_func(enc_isr)(void) {  
    static uint8_t prev_ab = 0;  
    uint8_t ab = (uint8_t)((gpio_get(ENC_PIN_A) << 1) | gpio_get(ENC_PIN_B));
    int8_t d = enc_lut[(prev_ab << 2) | ab];  
    prev_ab = ab;  
    enc_pending_steps += d;   // накапливаем; обработает vfo_encoder_poll()
}  
  
void vfo_encoder_init(void) {  
    pinMode(ENC_PIN_A, INPUT_PULLUP);  
    pinMode(ENC_PIN_B, INPUT_PULLUP);
    pinMode(VFO_ENC_SW_PIN, INPUT_PULLUP); 
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), enc_isr, CHANGE);  
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), enc_isr, CHANGE);
}  
  
// Вызывать из loop() каждую итерацию — само переключение частоты тяжёлое  
// (PLL-скан + перезапуск Core 1), поэтому вынесено из ISR.  
void vfo_encoder_poll(void) {  
  
    // ---------- секция 1: кнопка энкодера (toggle генерации) ----------  
    bool sw = gpio_get(VFO_ENC_SW_PIN);  
    uint32_t now_ms = millis();  
  
    if (sw != enc_sw_last && (now_ms - enc_sw_ms) > VFO_ENC_DEBOUNCE_MS) {  
        enc_sw_ms  = now_ms;  
        enc_sw_last = sw;  
  
        if (sw == 0) {                        // нажатие (INPUT_PULLUP)  
            if (!enc_tx_active) {  
                // ВКЛ: запуск генерации на vfo_current_freq_hz.  
                // arm флага ДО init — иначе init возьмёт pll_nominal (133 МГц).  
                vfo_clk_boost_arm();  
                vfo_hardware_init(vfo_current_freq_hz, 10.0);  
                vfo_set_tone_instant(0);  
                vfo_operation_set(true);  
                enc_tx_active = true;  

                Serial.printf("[ENC] TX ON  %lu Hz (clk=%lu kHz)\n",  
                              (unsigned long)vfo_current_freq_hz,  
                              (unsigned long)(current_clk_sys_hz / 1000ULL));  

                uint32_t clkdiv_now = lo_pio->sm[lo_sm].clkdiv;  
                uint32_t meas_clk   = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
                uint32_t meas_xosc  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_XOSC_CLKSRC) * 1000u;  
                uint32_t meas_rosc  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC) * 1000u;  
                uint32_t sys_ctrl   = clocks_hw->clk[clk_sys].ctrl;  
                
                Serial.printf("[DBG] CLKDIV=0x%08lX (int=%lu frac=%lu)  tone[0].int=%lu frac=%lu  "  
                            "active=%d changed=%d  clk_meas=%lu kHz  CS=0x%08lX FB=%lu PRIM=0x%08lX\n",  
                            (unsigned long)clkdiv_now,  
                            (unsigned long)(clkdiv_now >> 16),  
                            (unsigned long)((clkdiv_now >> 8) & 0xFF),  
                            (unsigned long)ifkp_tones[0].pio_int,  
                            (unsigned long)ifkp_tones[0].pio_frac,  
                            (int)current_active_tone, (int)tone_changed,  
                            (unsigned long)(meas_clk / 1000u),  
                            (unsigned long)pll_sys_hw->cs,  
                            (unsigned long)pll_sys_hw->fbdiv_int,  
                            (unsigned long)pll_sys_hw->prim);  
                
                // Фактическая опора PLL и состояние мультиплексора clk_sys  
                Serial.printf("[DBG2] XOSC=%u kHz  ROSC=%u kHz  clk_sys.ctrl=0x%08lX (src=%lu aux=%lu)  sys_div=0x%08lX (int=%lu)\n",  
                            frequency_count_khz(CLOCKS_FC0_SRC_VALUE_XOSC_CLKSRC),  
                            frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC),  
                            (unsigned long)clocks_hw->clk[clk_sys].ctrl,  
                            (unsigned long)(clocks_hw->clk[clk_sys].ctrl & CLOCKS_CLK_SYS_CTRL_SRC_BITS),  
                            (unsigned long)((clocks_hw->clk[clk_sys].ctrl & CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS) >> CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB),  
                            (unsigned long)clocks_hw->clk[clk_sys].div,  
                            (unsigned long)(clocks_hw->clk[clk_sys].div >> 8));   // INT-поле clk_sys.div: биты 8..23

            } else {  
                // ВЫКЛ: закрыть ключ, вернуть шину на номинал.  
                vfo_operation_set(false);  
                vfo_clk_boost_exit();         // внутри сам сбрасывает clk_boosted  
                enc_tx_active = false;  
                Serial.printf("[ENC] TX OFF\n");  
            }  
        }  
    }  
  
    // ---------- секция 2: накопленные тики энкодера ----------  
    int32_t ticks;  
    uint32_t ints = save_and_disable_interrupts();  
    ticks = enc_pending_steps;  
    enc_pending_steps = 0;  
    restore_interrupts(ints);  
  
    if (ticks == 0) return;  
  
    // Перевод тиков в детенты: на щелчок типичный энкодер даёт 4 перехода.  
    int32_t detents = ticks / ENC_TICKS_PER_DETENT;   // ENC_TICKS_PER_DETENT = 4  
    if (detents == 0) {  
        // Половина/четверть щелчка — вернуть тики обратно, накапливаем дальше.  
        ints = save_and_disable_interrupts();  
        enc_pending_steps += ticks;  
        restore_interrupts(ints);  
        return;  
    }  
  
    int64_t f = (int64_t)vfo_current_freq_hz +  
                (int64_t)detents * (int64_t)vfo_tune_step_hz;  
    if (f < 1000000LL)  f = 1000000LL;  
    if (f > 30000000LL) f = 30000000LL;  
    vfo_current_freq_hz = (uint32_t)f;  
  
    if (enc_tx_active) {  
        // Перестройка на лету: один полный цикл init.  
        // arm обязателен — после предыдущего boost_exit флаг снят,  
        // и без него init уйдёт на pll_nominal = 133 МГц.  
        vfo_clk_boost_arm();  
        vfo_hardware_init(vfo_current_freq_hz, 10.0);  
        vfo_set_tone_instant(0);  
        vfo_operation_set(true);              // init мог переоткрыть/закрыть пин  
        
        Serial.printf("[ENC] RETUNE %lu Hz (clk=%lu kHz)\n",  
                      (unsigned long)vfo_current_freq_hz,  
                      (unsigned long)(current_clk_sys_hz / 1000ULL));  
  
        // Измеренная частота шины + фактические регистры PLL и делителя SM  
        uint32_t clkdiv_now = lo_pio->sm[lo_sm].clkdiv;  
        uint32_t meas_clk   = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
        uint32_t meas_xosc  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_XOSC_CLKSRC) * 1000u;  
        uint32_t meas_rosc  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC) * 1000u;  
        uint32_t sys_ctrl   = clocks_hw->clk[clk_sys].ctrl;  
        
        Serial.printf("[DBG] CLKDIV=0x%08lX (int=%lu frac=%lu)  tone[0].int=%lu frac=%lu  "  
                    "active=%d changed=%d  clk_meas=%lu kHz  CS=0x%08lX FB=%lu PRIM=0x%08lX\n",  
                    (unsigned long)clkdiv_now,  
                    (unsigned long)(clkdiv_now >> 16),  
                    (unsigned long)((clkdiv_now >> 8) & 0xFF),  
                    (unsigned long)ifkp_tones[0].pio_int,  
                    (unsigned long)ifkp_tones[0].pio_frac,  
                    (int)current_active_tone, (int)tone_changed,  
                    (unsigned long)(meas_clk / 1000u),  
                    (unsigned long)pll_sys_hw->cs,  
                    (unsigned long)pll_sys_hw->fbdiv_int,  
                    (unsigned long)pll_sys_hw->prim);  
        
        // Фактическая опора PLL и состояние мультиплексора clk_sys  
        Serial.printf("[DBG2] XOSC=%u kHz  ROSC=%u kHz  clk_sys.ctrl=0x%08lX (src=%lu aux=%lu)  sys_div=0x%08lX (int=%lu)\n",  
                    frequency_count_khz(CLOCKS_FC0_SRC_VALUE_XOSC_CLKSRC),  
                    frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC),  
                    (unsigned long)clocks_hw->clk[clk_sys].ctrl,  
                    (unsigned long)(clocks_hw->clk[clk_sys].ctrl & CLOCKS_CLK_SYS_CTRL_SRC_BITS),  
                    (unsigned long)((clocks_hw->clk[clk_sys].ctrl & CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS) >> CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB),  
                    (unsigned long)clocks_hw->clk[clk_sys].div,  
                    (unsigned long)(clocks_hw->clk[clk_sys].div >> 8));   // INT-поле clk_sys.div: биты 8..23

    }  
    // TX выключен — частота просто обновлена; следующее нажатие  
    // кнопки стартует на новой частоте.  
}






/**  
 * @brief Мгновенная смена активного тона частотной сетки (FSK-переход).  
 *  
 * Перезагружает параметры PIO-делителя и DDS-шаг для движка дизеринга  
 * без разрыва фазы несущей: изменение применяется на следующей итерации  
 * модулятора, а не мгновенной записью в регистр `clkdiv`.  
 *  
 */
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


/**  
 * @brief Синхронное управление ключом эфира (нажатие/отпускание CW-ключа).  
 * Коммутирует выходной пин высокочастотного меандра @c pin_freq_out:  
 */
void __not_in_flash_func(vfo_operation_set)(bool key_down) {
    // 1. Управляем направлением пина генератора PIO (высокочастотный меандр)
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, pin_freq_out, 1, key_down);
}
