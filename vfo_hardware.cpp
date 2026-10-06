/**  
 * ============================================================================  
 *  vfo_hardware.cpp — Дизеринг-движок программного DDS VFO на автоматах PIO  
 *  Версия 2.15 (Профилирование и ASM-оптимизация Core 1), 2026-10-05  
 * ============================================================================ 
 * программная реализация MASH-2 (Multi-Stage Noise Shaping 2-го порядка
 * выполнен по схеме MASH 1-1 (каскад из двух последовательных дельта-сигма модуляторов 1-го порядка).
 * Конвейер полностью развернут внутри регистров процессора ARM Cortex-M0+ на изолированном ядре Core 1 
 * и работает без обращения к ОЗУ. Благодаря этому частота расчета дизеринга 
 * достигает своего физического потолка — нескольких мегагерц.

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
 
 *     и делители (p1, p2). Подбирает частоту тактирования clk_sys (100..133 МГц)  
 *     так, чтобы минимизировать остаток 32-битного шага dds_step (первично)  
 *     и остаток pio_frac (вторично).  
 *  2. Дизеринг на Core 1: Высокоскоростной регистровый Си-конвейер (VFO_DITHER_FAST)  
 *     работает в бесконечном цикле на изолированном ядре. Каждую итерацию  
 *     производится расчет Delta-Sigma модуляции 2-го порядка (MASH-1-1)  
 *     с инъекцией 4-битного псевдослучайного шума (Xorshift32) для декорреляции  
 *     спектра и подавления побочных спуров. Результат атомарно пишется напрямую  
 *     в регистр clkdiv автомата PIO.  
 *  
 *  ДВА ЯДРА И ГОРЯЧИЙ ЦИКЛ  
 *  При VFO_DITHER_ON_CORE1 дизеринг крутится в бесконечном цикле на Core 1  
 *  (vfo_core1_entry). Единственная запись за итерацию — STR в регистр clkdiv  
 *  PIO через шину периферии; она вместе с переходом цикла задаёт нижний  
 *  предел периода, поэтому потолок F_s_dither при clk_sys ~130 МГц — единицы  
 *  МГц. Ветка VFO_DITHER_FAST держит все состояния в регистрах  
 *  ради максимальной F_s_dither; эталонная медленная Си-ветка (else) прогоняет  
 *  состояние через глобальное ОЗУ и служит для сверки.  
 *  
 *  СИНХРОНИЗАЦИЯ  
 *  Обмен Core0<->Core1 (dds_step, target_pio_int/frac8, tone_changed) защищён  
 *  аппаратным спинлоком vfo_spin_lock. Флаг tone_changed опрашивается каждую  
 *  итерацию, но спинлок берётся только в момент фактической смены тона.  
 *  Все функции горячего пути помечены __not_in_flash_func и работают с целыми.  
 *  
 *  ЗНАКОВАЯ НОРМАЛИЗАЦИЯ (критично)  
 *  total_correction MASH-2 может быть отрицательным (-1). current_frac/current_int  
 *  строго int32_t: применяется арифметический сдвиг (asrs) и маска 0xFF, иначе  
 *  логический сдвиг испортил бы clkdiv на итерациях с отрицательной коррекцией.  
 *  
 *  СМЕНА ЧАСТОТЫ / ТОНА  
 *  Смена тона внутри сессии (RTTY mark/space, IFKP) не перезапускает Core 1 и  
 *  не трогает clk_sys — F_s_dither постоянна. Смена базовой частоты диапазона  
 *  = полный цикл reset -> переконфигурация clk_sys (pll_init/clock_configure) ->  
 *  relaunch Core 1; при этом F_s_dither меняется вместе с выбранной clk_sys.  
 *  Отдельной регулируемой «частоты шины» нет — периферия тактируется от clk_sys.  
 *  
 *  ПРЕДЕЛЫ СПЕКТРА  
 *  Дизеринг и автотюн чистят ближнюю зону только пока делитель D достаточно  
 *  велик. У D ~= 2 (10 м) шаг сетки ~= десятки кГц неустраним параметрами софта.  
 *  Дальние горбы MASH и гармоники меандра — задача аналогового выходного ФНЧ.  
 * ============================================================================  
 */

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/resets.h"
#include "hardware/timer.h"
#include "hardware/structs/sio.h"
#include "hardware/vreg.h"
#include <hardware/watchdog.h>
#include <hardware/adc.h>
#include "hardware/divider.h"
#include "hardware/structs/ssi.h"   // ssi_hw->baudr — делитель XIP_SSI
#include "pico/multicore.h"
#include "vfo_hardware.h"
#include "file_manager.h"


// ============================================================================
// ПРОТОТИПЫ (ОБЪЯВЛЕНИЯ) ВНУТРЕННИХ ФУНКЦИЙ ФАЙЛА
// ============================================================================
static void detach_peripheral_clock();
static void __not_in_flash_func(vfo_set_clk_sys)(const PllConfig& cfg, uint32_t vsel);
static VfoParameters calculate_raw_params_mhz(uint64_t clk_sys_hz, uint64_t mhz_target);
PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz, uint64_t max_clk_limit);


// Добавлен упреждающий прототип Core 1, теперь vfo_clk_boost_enter сможет его вызвать!
static void __not_in_flash_func(vfo_core1_entry)();

static void vfo_rebuild_tone_table(uint64_t base_freq_mhz, uint64_t step_mhz); 
static vreg_voltage vsel_for(uint64_t clk_hz);
static uint64_t vfo_base_mhz = 0;   // базовый тон сетки, мГц  
static uint64_t vfo_step_mhz = 0;   // шаг сетки, мГц

// Глобальные профили тактирования (static полностью удалены для extern-связывания)
// ИСПРАВЛЕНО: Теперь номинал знает про калибровку кварца и равен честным 138 МГц.
// Это исключит ложные срабатывания фильтров "opt <= nominal".
PllConfig pll_nominal   = { 133, 6, 2, 133000000ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false };
PllConfig pll_overclock = { 69,  3, 2, 138004025ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false }; 
// Потолок OCTEST клэмпим строго к лимиту препроцессора (380 МГц)
PllConfig pll_ceiling   = { 95,  3, 1, 380011083ULL, (uint32_t)VREG_VOLTAGE_1_30,    true };  
volatile bool clk_boosted = false;       // активация разгона

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
 *  
 * Это обязательный шаг при ретюнинге PLL: когда `pll_sys`  
 * сбрасывается (`reset_block`/`unreset_block_wait`) и меняет частоту  
 * clk_sys, периферия, жёстко привязанная к системной шине, получила бы  
 * скачок тактовой частоты и сбой (потеря baud rate UART, срыв USB).  
 * Детач гарантирует стабильные 12 МГц для clk_peri на всём интервале  
 * переконфигурации.  
 *  
 * @note Вызывается из @ref vfo_hardware_init непосредственно перед  
 *       `save_and_disable_interrupts()` и каскадом переинициализации  
 *       `pll_sys`. После возврата clk_sys на PLL периферийный домен  
 *       остаётся на XOSC — это осознанно: clk_peri не зависит от  
 *       автотюна системной частоты.  
 * @note Оперирует регистрами подсистемы CLOCKS/PLL напрямую;  
 *       не является thread-safe вне критической секции инициализации.  
 *  

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
 * @param cfg   Конфигурация PLL: fbdiv, p1, p2, clk_sys_hz, vsel, is_oc  
 * @param vsel  Целевое напряжение ядра (например, VREG_VOLTAGE_1_20)  
 *  
 * @note Смена BAUDR выполняется ПОКА clk_sys ещё на XOSC (12 МГц) —  
 *       регистр пишется безопасно; после возврата clk_sys на PLL_SYS  
 *       флэш-контроллер уже работает на новом делителе.  
 */  
static void __not_in_flash_func(vfo_set_clk_sys)(const PllConfig& cfg, uint32_t vsel) {  
    uint32_t target_clk_hz = (uint32_t)(((uint64_t)cfg.fbdiv * VFO_CALIBRATED_XOSC_HZ) / (uint64_t)(cfg.p1 * cfg.p2));  
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
  
    uint32_t vco_nominal_hz = (uint32_t)(VFO_CALIBRATED_XOSC_HZ * (uint64_t)cfg.fbdiv);  
    pll_init(pll_sys, 1, vco_nominal_hz, cfg.p1, cfg.p2);  
  
    // === BAUDR: SSI принимает запись ТОЛЬКО при SSIENR=0 ===  
    // Мы на XOSC, код в SRAM, прерывания запрещены — окно безопасно.  
    uint32_t ssi_baud = (uint32_t)((target_clk_hz + VFO_FLASH_SCK_MAX_HZ - 1) / VFO_FLASH_SCK_MAX_HZ);  
    ssi_baud = (ssi_baud + 1u) & ~1u;  
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
    current_clk_sys_hz = target_clk_hz;  
  
    if (!is_overclocking) {  
        busy_wait_us(100);  
        vreg_set_voltage((vreg_voltage)vsel);  
    }  
}




void vfo_clk_boost_enter(unsigned int target_freq_hz) {  
    if (clk_boosted) return;
    if (target_freq_hz != 0) {
  
    // 1. Поиск оптимума сразу в пределах доказанного потолка (один проход,  
    //    двойное сканирование с клэмпом больше не нужно)  
    PllConfig opt = vfo_find_optimal_pll(target_freq_hz, VFO_CLK_SYS_MAX_HZ);  
  
    // 2. Сравнение метрик по ЕДИНОЙ формуле поиска:  
    //    едем на opt только если он реально лучше текущей шины.  
    //    Разрешены оба направления: opt > nominal (разгон) и opt < nominal  
    //    (чистая шина ниже 133 МГц).  
    uint64_t target_mhz = (uint64_t)target_freq_hz * 1000ULL;  

    VfoParameters dummy;  
    uint64_t metric_opt = vfo_pll_metric(opt.clk_sys_hz,     target_mhz, &dummy);  
    uint64_t metric_cur = vfo_pll_metric(current_clk_sys_hz, target_mhz, &dummy);

  
    if (metric_opt >= metric_cur || opt.clk_sys_hz == current_clk_sys_hz) {  
        Serial.printf("[BOOST] SKIP (opt %llu.%03llu MHz не лучше: metric=%llu vs active=%llu)\n",  
                      (unsigned long long)(opt.clk_sys_hz / 1000000ULL),  
                      (unsigned long long)(opt.clk_sys_hz % 1000000ULL / 1000ULL),  
                      (unsigned long long)metric_opt,  
                      (unsigned long long)metric_cur);  
        return;  
    }  
  
    // 3. Теперь, когда решение принято — останавливаем Core 1  
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
  
    Serial.printf("[BOOST] ON   clk_sys=%7.3f MHz  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)\n",  
                  opt.clk_sys_hz / 1000000.0,  
                  (unsigned long)opt.fbdiv,  
                  (unsigned long)opt.p1,  
                  (unsigned long)opt.p2,  
                  (unsigned long)(opt.fbdiv * (uint32_t)(VFO_CALIBRATED_XOSC_HZ / 1000000ULL)),  
                  (unsigned)vsel_to_mv(vsel),  
                  opt.clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,  
                  (unsigned)ssi_hw->baudr);  
    }

    clk_boosted = true;
}






  
/**  
 * @brief Выход из boost: возврат на pll_nominal и штатное напряжение.  
 * Понижение VREG выполняется ПОСЛЕ снижения частоты (логика в vfo_set_clk_sys).  
 */  
void vfo_clk_boost_exit(void) {  
    if (!clk_boosted) return;  
  
    vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);

    vfo_rebuild_tone_table(vfo_base_mhz, vfo_step_mhz);
    Serial.println("[BOOST] пересчёт таблицы тонов");
  
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
    Serial.printf("[BOOST] OFF  clk_sys=%7.3f MHz  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)  T_CPU=%5.1f C\n",  
                  pll_nominal.clk_sys_hz / 1000000.0,  
                  (unsigned long)pll_nominal.fbdiv,  
                  (unsigned long)pll_nominal.p1,  
                  (unsigned long)pll_nominal.p2,  
                  (unsigned long)(pll_nominal.fbdiv * (uint32_t)(VFO_CALIBRATED_XOSC_HZ / 1000000ULL)),  
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
 * @param[in] state  Текущее 32-битное состояние ГПСЧ (не равное 0 —  
 *                   нулевое состояние вырождает генератор в постоянный 0).  
 *  
 * @return Следующее псевдослучайное состояние; из него выделяются  
 *         младшие биты для дизеринга шага DDS  
 *         (см. `VFO_DITHER_RANDOMIZE` в @ref vfo_dither_step и  
 *         @ref vfo_core1_entry).  
 *  
 * @note Помечена `always_inline` + `__not_in_flash_func` — разворачивается  
 *       в 3 пары LSL/LSR+EOR в месте вызова, исполняется из ОЗУ.  
 *       Полный период генератора 2^32−1 при ненулевом стартовом состоянии  
 *       (@ref VFO_RAND_SEED_INIT, пересеивается от `time_us_32()` в  
 *       @ref vfo_hardware_init).  
 * @note Вариант с глобальным состоянием ОЗУ — @ref vfo_xorshift32(),  
 *       используется в медленной эталонной ветке @ref vfo_dither_step.  
 *  
 * @see vfo_xorshift32(), vfo_dither_step(), vfo_core1_entry()  
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
 * @return Следующее псевдослучайное значение; младшие биты  
 *         используются как шум дизеринга шага DDS  
 *         (см. `VFO_DITHER_RANDOMIZE` в @ref vfo_dither_step).  
 *  
 * @note В отличие от чистой версии @ref vfo_xorshift32_raw(),  
 *       обращается к глобальному ОЗУ — используется только в медленной  
 *       эталонной ветке @ref vfo_dither_step (режим Core 0 и отладочная  
 *       ветка Core 1). В быстром регистровом конвейере  
 *       @ref vfo_core1_entry применяется инлайн-версия с локальным  
 *       состоянием @c loc_rand_state, синхронизируемым с  
 *       @c xorshift_state при смене тона.  
 * @note Помечена `always_inline` + `__not_in_flash_func` — исполняется  
 *       из ОЗУ. Стартовое состояние — @ref VFO_RAND_SEED_INIT,  
 *       пересеивается от `time_us_32()` в @ref vfo_hardware_init;  
 *       нулевое состояние вырождает генератор в постоянный 0.  
 *  
 * @see vfo_xorshift32_raw(), vfo_dither_step(), vfo_core1_entry()  
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
 * Зависит от компайл-тайм опций:  
 * - `VFO_USE_MASH2`: двухступенчатый конвейер MASH 1-1 — вторая ступень  
 *   интегрирует состояние первой, перенос второй дифференцируется  
 *   (total_correction = carry1 + carry2 − carry2_prev, диапазон −1..+2);  
 *   иначе — одноступенчатый DDS с коррекцией 0/+1;  
 * - `VFO_DITHER_RANDOMIZE`: к шагу подмешивается 4-битный шум Xorshift32  
 *   с мат. ожиданием 0 для декорреляции спектра и подавления спуров.  
 *  
 * Отрицательная коррекция нормализуется циклами заёма/переноса между  
 * целой и дробной частями делителя (строгая знаковая арифметика int32_t).  
 *  
 * @param[in] local_step  32-битный шаг DDS (дробное приращение частоты,  
 *                        остаток делителя из @ref calculate_raw_params_mhz).  
 * @param[in] local_int   Целая часть базового делителя PIO (INT, >= 2).  
 * @param[in] local_frac  Дробная часть базового делителя PIO (FRAC8, 0..255).  
 *  
 * @note Состояние (аккумуляторы, ГПСЧ) хранится в глобальном ОЗУ  
 *       (@c dds_accumulator, @c dds_accum_m2, @c m2_carry_prev,  
 *       @c xorshift_state) — эта ветка медленнее регистрового конвейера  
 *       `VFO_DITHER_FAST` и служит эталоном для сверки.  
 * @note Помечена `__not_in_flash_func` — исполняется из ОЗУ.  
 *       Вызывается из `vfo_dither_callback` (режим Core 0) и из медленной  
 *       ветки `vfo_core1_entry`.  
 *  
 * @see vfo_core1_entry(), vfo_dither_callback(), vfo_set_tone_instant()  
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
 * Вызывается повторяющимся таймером @c sdr_dither_timer с периодом  
 * `VFO_DITHER_INTERVAL_US` (зарегистрирован через `add_repeating_timer_us`  
 * в @ref vfo_hardware_init). Каждый вызов делегирует одну итерацию  
 * Delta-Sigma модуляции функции @ref vfo_dither_step, передавая текущие  
 * параметры тона из глобальных переменных ОЗУ (@c dds_step,  
 * @c target_pio_int, @c target_pio_frac8), которые обновляет  
 * @ref vfo_set_tone_instant под `save_and_disable_interrupts`.  
 *  
 * @param[in] t  Указатель на структуру повторяющегося таймера SDK  
 *               (не используется).  
 *  
 * @return Всегда @c true — таймер продолжает срабатывать с тем же периодом.  
 *  
 * @note Компилируется только когда `VFO_DITHER_ON_CORE1` не определён —  
 *       в режиме Core 1 модуляцию выполняет горячий цикл  
 *       @ref vfo_core1_entry, а этот колбэк и таймер отсутствуют.  
 * @note Помечена `__not_in_flash_func` — исполняется из ОЗУ в контексте  
 *       прерывания таймера; частота дизеринга ограничена периодом таймера  
 *       и существенно ниже, чем у регистрового конвейера Core 1.  
 *  
 * @see vfo_dither_step(), vfo_set_tone_instant(), vfo_hardware_init(),  
 *      vfo_core1_entry()  
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
 * Запускается через `multicore_launch_core1()` и выполняет Delta-Sigma  
 * модуляцию (MASH 1-1 при `VFO_USE_MASH2`, иначе одноступенчатый DDS) на  
 * максимально возможной скорости: единственная операция с периферией за  
 * итерацию — запись результата в регистр `clkdiv` автомата PIO через  
 * кэшированный указатель @c clkdiv_reg. Частота итераций (F_s_dither)  
 * определяется только @c clk_sys и достигает единиц МГц.  
 *  
 * Две компайл-тайм ветки:  
 * - `VFO_DITHER_FAST` — регистровый конвейер без обращения к ОЗУ: все  
 *   состояния (аккумуляторы @c loc_acc1/@c loc_acc2, перенос  
 *   @c loc_m2_carry_prev, ГПСЧ @c loc_rand_state) живут в регистрах;  
 *   переносы извлекаются через `__builtin_add_overflow`, нормализация  
 *   делителя — арифметическим сдвигом (asrs) + маска 0xFF;  
 * - `else` — эталонная медленная ветка: прогоняет состояние через  
 *   глобальные переменные ОЗУ вызовом @ref vfo_dither_step; служит для  
 *   сверки и отладки.  
 *  
 * При `VFO_DITHER_RANDOMIZE` к шагу подмешивается шум Xorshift32  
 * (ряд −3..+3, мат. ожидание 0) для размывания спектральных спуров.  
 *  
 * Синхронизация с Core 0: флаг @c tone_changed опрашивается каждую  
 * итерацию; при смене тона параметры (@c dds_step, @c target_pio_int,  
 * @c target_pio_frac8, @c xorshift_state) перечитываются под аппаратным  
 * спинлоком @c vfo_spin_lock, аккумуляторы сбрасываются.  
 *  
 * Опция `VFO_DITHER_PROFILE` включает тоггл отладочного пина через SIO  
 * для измерения реальной F_s_dither осциллографом/анализатором.  
 *  
 * @note Функция не возвращается. Помечена `__not_in_flash_func` —  
 *       исполняется из ОЗУ. Компилируется только при `VFO_DITHER_ON_CORE1`;  
 *       иначе дизеринг обслуживает таймерный колбэк  
 *       @ref vfo_dither_callback на Core 0.  
 *  
 * @see vfo_dither_step(), vfo_dither_callback(), vfo_set_tone_instant()  
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









/**  
 * @brief Прецизионный расчёт параметров PIO-делителя для целевой частоты  
 *        в миллигерцах (0.001 Гц).  
 *  
 * Вычисляет делитель автомата PIO в формате fixed-point Q8.8  
 * (pio_int + pio_frac/256) для выходного меандра `f_out = clk_sys / (2 * D)`,  
 * а также 32-битный остаток ошибки `dds_step` — приращение для DDS/MASH-2  
 * дизеринга, компенсирующее остаточную дробную часть делителя.  
 *  
 * Вся арифметика 64-битная целочисленная; расчёт в миллигерцах с  
 * масштабирующим коэффициентом 1000 исключает накопление ошибки округления  
 * для шага сетки тонов (RTTY mark/space, IFKP).  
 *  
 * @param[in] clk_sys_hz   Текущая системная частота в герцах (результат  
 *                           
 * @param[in] mhz_target   Целевая выходная частота в миллигерцах;  
 *                         автоматически ограничивается диапазоном  
 *                         КВ-диапазона 1.0..40.0 МГц.  
 *  
 * @return Структура @ref VfoParameters:  
 *         - @c pio_int — целая часть делителя (>= 2; при < 2 принудительно  
 *           зажимается в 2, @c pio_frac = 0);  
 *         - @c pio_frac — дробная часть делителя FRAC8 (0..255);  
 *         - @c dds_step — 32-битный шаг DDS-остатка для  
 *           @ref vfo_dither_step / регистрового конвейера Core 1;  
 *         - @c target_freq_chz — целевая частота, приведённая обратно в  
 *           сантигерцы для совместимости со структурой.  
 *  
 * @note Функция чистая: не обращается к периферии и не изменяет глобальное  
 *       состояние. Остаток `dds_step` вычисляется двухступенчатым делением  
 *       сдвинутого остатка (`clk_sys_rem << 16`) для сохранения полных  
 *       32 бит точности без потерь при делении.  
 *  
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




#define VFO_PLL_DEBUG            1         // 0 — выключить отладочную печать кандидатов  
#define VFO_DITHER_LOOP_CYCLES   18ULL     // тактов на итерацию Core 1  
#define VFO_DITHER_SPUR_GUARD_HZ 2000ULL   // защитное окно в Гц  
#define VFO_SPUR_NUMERATOR (VFO_DITHER_SPUR_GUARD_HZ * VFO_DITHER_LOOP_CYCLES * 4294967296ULL)  
#define VFO_FRAC_PENALTY         200000ULL // вес штрафа за единицу frac-отклонения (PIO-джиттера)
#define VFO_SPUR_PENALTY         400000000ULL  

// ========================================================================  
// ЕДИНАЯ МЕТРИКА КАНДИДАТА — единственный источник истины.  
// чтобы сравнение "opt vs active" было в той же шкале, что и поиск.  
// target_mhz — частота в миллигерцах (Hz * 1000).  
// ========================================================================  
static uint64_t vfo_pll_metric(uint64_t clk_sys_hz, uint64_t target_mhz,  
                               VfoParameters *out /* может быть NULL */) {  
    VfoParameters test = calculate_raw_params_mhz(clk_sys_hz, target_mhz);  
    if (out) *out = test;  
  
    uint64_t metric;  

    if (test.pio_frac == 0 && test.dds_step == 0) {  
        metric = 0;   // аппаратный идеал: целое деление без дизера  
    } else {  
        // Расстояние шага до 0 или 2^32 для 1-й, 2-й и 3-й гармоник спура  
        uint32_t e1 = test.dds_step;  
        uint32_t e2 = test.dds_step << 1;  
        uint32_t e3 = test.dds_step + (test.dds_step << 1);  
  
        uint32_t d1 = (e1 < (uint32_t)(4294967296ULL - e1)) ? e1 : (uint32_t)(4294967296ULL - e1);  
        uint32_t d2 = (e2 < (uint32_t)(4294967296ULL - e2)) ? e2 : (uint32_t)(4294967296ULL - e2);  
        uint32_t d3 = (e3 < (uint32_t)(4294967296ULL - e3)) ? e3 : (uint32_t)(4294967296ULL - e3);  
  
        uint32_t dmin = d1;  
        if (d2 < dmin) dmin = d2;  
        if (d3 < dmin) dmin = d3;  
        // метрика ∝ сдвиг спура в Гц (с запасом по разрядности)  
        metric = ((uint64_t)dmin * clk_sys_hz) >> 18;   // ~ dmin·clk/262144
  
        // Прогрессивный штраф за спур в защитном окне у несущей  
        uint64_t thr  = VFO_SPUR_NUMERATOR / clk_sys_hz;  
        uint64_t dead = thr / 20;   // спур ближе ~100 Гц к тону — сливается с несущей
                                    // thr/8 ~ 250 Гц
        if ((uint64_t)dmin > dead && (uint64_t)dmin < thr) {  
            uint64_t prox = thr - dmin;  
            metric += VFO_SPUR_PENALTY + prox * VFO_SPUR_PENALTY / thr;  
        }
    }  

    // Малый pio_int: DDS-остаток модулирует слишком большую долю периода.  
    // INT=2 с дизером — глубокая модуляция, метрика её недооценивает.  
    if (test.pio_int < 4) metric += 500000000ULL;   // или return UINT64_MAX — отсечь совсем

    // Аддитивный штраф за PIO-джиттер (расстояние frac до 0/256)  
    uint32_t fdist = (test.pio_frac < 256u - test.pio_frac)  
                     ? test.pio_frac : 256u - test.pio_frac;  
    metric += (uint64_t)fdist * VFO_FRAC_PENALTY * 133000000ULL / clk_sys_hz;
  
    return metric;  
}
  


PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz, uint64_t max_clk_limit) {  

    uint64_t crystal_hz = VFO_CALIBRATED_XOSC_HZ;  
    uint64_t target_mhz = (uint64_t)target_frequency_hz * 1000ULL; // миллигерцы  
  
    // Дефолтная безопасная конфигурация на случай сбоя  
    PllConfig best_pll = { 133, 6, 2, 133000000ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false };  
  
    // Защитный клэмпинг диапазона (1.0 .. 40.0 МГц)  
    if (target_mhz < 100000000ULL || target_mhz > 40000000000ULL) {  
        return best_pll;  
    }  
  
    // Эталон — фактический pll_nominal через ту же метрику  
    VfoParameters nom_params;  
    uint64_t nom_dds_metric = vfo_pll_metric(pll_nominal.clk_sys_hz, target_mhz, &nom_params);  
  
    uint32_t nom_dist_frac = (nom_params.pio_frac < 256u - nom_params.pio_frac)  
                             ? nom_params.pio_frac : 256u - nom_params.pio_frac;  
    uint32_t nom_raw_step  = nom_params.dds_step & ~1u; // сырая чётность для CTZ  
  
    // Стартовые ориентиры от номинала  
    uint64_t min_dds_metric  = nom_dds_metric;  
    uint32_t min_ctz_metric  = (nom_raw_step == 0) ? 0u : (uint32_t)__builtin_ctz(nom_raw_step);  
    uint32_t min_frac_metric = nom_dist_frac;  
  
#if VFO_PLL_DEBUG  
    struct PllCand { uint64_t clk; uint32_t step; uint64_t metric; uint32_t frac; };  
    PllCand top[3] = { {0,0,UINT64_MAX,0}, {0,0,UINT64_MAX,0}, {0,0,UINT64_MAX,0} };  
#endif  
  
    // Сканируем весь диапазон в один проход  
    const uint64_t min_allowed_clk = VFO_CLK_SYS_MIN_HZ;  
    const uint64_t max_allowed_clk = max_clk_limit;  

    Serial.printf("[PLLDBG] scan range: %.3f .. %.3f MHz (target=%u)\n",  
              min_allowed_clk / 1e6, max_allowed_clk / 1e6, target_frequency_hz);

    for (uint32_t p1 = 2; p1 <= 6; p1++) {  
        for (uint32_t p2 = 1; p2 <= 2; p2++) {  
            uint32_t pdiv_total = p1 * p2;  
  
            for (uint32_t fbdiv = 30; fbdiv <= 150; fbdiv++) {  
                uint64_t vco_hz = fbdiv * crystal_hz;  
  
                // Жесткий аппаратный фильтр VCO RP2040 (750..1600 МГц)  
                if (vco_hz < 750000000ULL || vco_hz > 1600000000ULL) continue;  
  
                uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;  
                if (clk_sys_hz < min_allowed_clk || clk_sys_hz > max_allowed_clk) continue;  
  
                // Единая метрика кандидата  
                VfoParameters test;  
                uint64_t current_dds_metric = vfo_pll_metric(clk_sys_hz, target_mhz, &test);  
                if (test.pio_int < 2) continue;  
  
                // CTZ по сырому шагу (сброс принудительной нечётности)  
                uint32_t cand_raw_step = test.dds_step & ~1u;  
                uint32_t current_ctz_metric = (cand_raw_step == 0) ? 0u  
                                              : (uint32_t)__builtin_ctz(cand_raw_step);  
  
                uint32_t current_frac_metric = (test.pio_frac < 256u - test.pio_frac)  
                                               ? test.pio_frac : 256u - test.pio_frac;  
  
#if VFO_PLL_DEBUG  
                // топ-3 без дублей по clk_sys (одна частота находится через разные p1/p2)  
                bool dup = (top[0].clk == clk_sys_hz) || (top[1].clk == clk_sys_hz);  
                if (!dup && current_dds_metric < top[2].metric) {  
                    int pos = (current_dds_metric < top[0].metric) ? 0 :  
                              (current_dds_metric < top[1].metric) ? 1 : 2;  
                    for (int k = 2; k > pos; k--) top[k] = top[k-1];  
                    top[pos].clk    = clk_sys_hz;  
                    top[pos].step   = test.dds_step;  
                    top[pos].metric = current_dds_metric;  
                    top[pos].frac   = test.pio_frac;  
                }  
#endif  
  
                // Многокритериальный арбитраж: dds -> ctz -> frac -> выше clk_sys  
                bool is_better_dds  = (current_dds_metric < min_dds_metric);  
                bool is_equal_dds   = (current_dds_metric == min_dds_metric);  
                bool is_better_ctz  = (current_ctz_metric < min_ctz_metric);  
                bool is_equal_ctz   = (current_ctz_metric == min_ctz_metric);  
                bool is_better_frac = (current_frac_metric < min_frac_metric);  
                bool is_equal_frac  = (current_frac_metric == min_frac_metric);  
  
                if (is_better_dds ||  
                   (is_equal_dds && is_better_ctz) ||  
                   (is_equal_dds && is_equal_ctz && is_better_frac) ||  
                   (is_equal_dds && is_equal_ctz && is_equal_frac && clk_sys_hz > best_pll.clk_sys_hz)) {  
  
                    min_dds_metric  = current_dds_metric;  
                    min_ctz_metric  = current_ctz_metric;  
                    min_frac_metric = current_frac_metric;  
  
                    best_pll.fbdiv = fbdiv;  
                    best_pll.p1 = p1;  
                    best_pll.p2 = p2;  
                    best_pll.clk_sys_hz = clk_sys_hz;  
                    best_pll.vsel = (uint32_t)vsel_for(clk_sys_hz); 
                    best_pll.is_oc = (clk_sys_hz > VFO_CLK_SYS_NOMINAL_HZ);  
                }  
            }  
        }  
    }  
  
#if VFO_PLL_DEBUG  
    Serial.printf("[PLLDBG] target=%u Hz  nominal: clk=%lu step=0x%08lX metric=%llu frac=%u\n",  
                  target_frequency_hz,  
                  (unsigned long)pll_nominal.clk_sys_hz,  
                  (unsigned long)nom_params.dds_step,  
                  (unsigned long long)nom_dds_metric,  
                  (unsigned)nom_params.pio_frac);  
    for (int i = 0; i < 3; i++) {  
        if (top[i].clk == 0) break;  
        Serial.printf("[PLLDBG] top%d: clk=%.3f MHz step=0x%08lX metric=%llu frac=%u\n",  
                      i, top[i].clk / 1e6, (unsigned long)top[i].step,  
                      (unsigned long long)top[i].metric, (unsigned)top[i].frac);  
    }  
    Serial.printf("[PLLDBG] WINNER: clk=%.3f MHz fbdiv=%lu p1=%lu p2=%lu\n",  
                  best_pll.clk_sys_hz / 1e6,  
                  (unsigned long)best_pll.fbdiv,  
                  (unsigned long)best_pll.p1,  
                  (unsigned long)best_pll.p2);  
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
 * vfo_clk_boost_enter() уже был активирован диспетчером сессий
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
        target_pll = vfo_find_optimal_pll(base_freq_hz, VFO_CLK_SYS_MAX_HZ); 
        
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
        
        Serial.printf("[BOOST] ON   clk_sys=%7.3f MHz  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)  T_CPU=%5.1f C\n",    
                      (double)current_clk_sys_hz / 1000000.0,  // Выведет честные 254.007 MHz!
                      (unsigned long)target_pll.fbdiv,    
                      (unsigned long)target_pll.p1,    
                      (unsigned long)target_pll.p2,    
                      (unsigned long)(target_pll.fbdiv * (uint32_t)(VFO_CALIBRATED_XOSC_HZ / 1000000ULL)),    
                      (unsigned)vsel_to_mv(selected_vsel),    
                      (double)current_clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,    
                      (unsigned)ssi_hw->baudr,    
                      (double)vfo_read_core_temp_c());
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

#ifdef VFO_SNAP_TO_GRID
    uint32_t min_dds_metric = 0xFFFFFFFFu;
    for (int32_t offset_chz = -10; offset_chz <= 10; offset_chz++) {
        uint64_t candidate_chz = (uint64_t)((int64_t)base_target_chz + offset_chz);
        // Математика calculate_raw_params_mhz жестко опирается на РЕАЛЬНУЮ clk_sys_hz
        VfoParameters candidate_params = calculate_raw_params_mhz(current_clk_sys_hz, candidate_chz * 10ULL);
        uint32_t dstep = candidate_params.dds_step; 
        uint32_t dist_to_0 = dstep; 
        uint32_t dist_to_max = 0xFFFFFFFFu - dstep;
        uint32_t current_metric = (dist_to_0 < dist_to_max) ? dist_to_0 : dist_to_max;
        
        if (current_metric < min_dds_metric) { 
            min_dds_metric = current_metric; 
            snapped_base_chz = candidate_chz; 
        }
    }
#endif

    vfo_base_mhz = snapped_base_chz * 10ULL; 
    vfo_step_mhz = (uint64_t)(step_hz * 1000.0);  
    
    // Синхронный потокобезопасный расчет всей FSK/IFKP таблицы тонов
    vfo_rebuild_tone_table(vfo_base_mhz, vfo_step_mhz); 
    vfo_set_tone_instant(0);

    // Диагностический вывод рантайм-телеметрии в Serial
    if (debug_flag) {
        VfoParameters real_base_params = ifkp_tones[0];
        Serial.printf("\n--- VFO Runtime Diagnostics ---\n");
        Serial.printf("Target Freq: %u Hz (Grid: %.2f Hz)\n", base_freq_hz, (double)real_base_params.target_freq_chz / 100.0);
        Serial.printf("clk_sys    : %u Hz (Физический захват PLL: fbdiv=%lu, p1=%lu, p2=%lu)\n", 
                      current_clk_sys_hz, (unsigned long)target_pll.fbdiv, (unsigned long)target_pll.p1, (unsigned long)target_pll.p2);
        Serial.printf("PIO Regs   : INT=%u, FRAC=%u\n", real_base_params.pio_int, real_base_params.pio_frac);
        Serial.printf("DDS Step   : 0x%08X (%u)\n", real_base_params.dds_step, real_base_params.dds_step);
        Serial.printf("-------------------------------\n");
    }

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
static float vfo_read_core_temp_c(void) {
    // Восстановление clk_adc после смены clk_sys (48 МГц от PLL_USB)  
    clock_configure(clk_adc,  
                    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
                    0, 48000000u, 48000000u);
    adc_select_input(4);  
    uint16_t raw = adc_read();  
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
        vfo_rebuild_tone_table(cached_base_freq_hz, cached_step_hz);  
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
        vfo_rebuild_tone_table(cached_base_freq_hz, cached_step_hz);  
        vfo_set_tone_instant(0);  
        clk_boosted = true;  
        thermal_throttled = false;  
        Serial.printf("[BOOST] RESUME: T_CPU=%.1f C — возврат на %7.3f MHz\n",  
                      (double)temp, (double)pll_overclock.clk_sys_hz / 1000000.0);  
    }  
}



/**  
 * @brief Стресс-тест: поиск индивидуального потолка clk_sys кристалла.  
 *  
 * Перебирает валидные PLL-комбинации по нарастающей clk_sys  

 * поднимает VSEL по таблице, переключает clk_sys через vfo_set_clk_sys,  
 * прогоняет композитную нагрузку (целочисленная арифметика + float,  
 * запись/чтение SRAM с CRC, непредсказуемые ветвления, аппаратный  
 * делитель SIO) и при успехе фиксирует ступень в pll_overclock.  
 *  
 * Функция размещена в SRAM (__not_in_flash_func): код и стековая  
 * нагрузка теста не обращаются к XIP/Flash, поэтому тест измеряет  
 * именно предел ядра, а не QSPI. Печать через Serial остаётся в Flash,  
 * но она выполняется между фазами и не входит в нагрузочный прогон.  
 *  
 * Scratch-регистр watchdog хранит частоту (МГц) текущей ступени:  
 * при зависании и ребуте setup() может прочитать сбойную частоту.  
 *  
 * @warning Тест разрушителен: зависание на сбойной ступени — нормальный  
 *          исход. Вызывать только при is_transmitting == false.  
 */  
void __not_in_flash_func(vfo_find_max_stable_clock)(void) {  
    const uint64_t CLK_START_HZ = VFO_CLK_SYS_NOMINAL_HZ;  
    const uint64_t CLK_CEIL_HZ  = VFO_CLK_SYS_MAX_HZ;  
  
    // Scratch-регистр watchdog: переживает зависание и ребут —  
    // содержит частоту (МГц) ступени, на которой ядро упало  
    volatile uint32_t *scratch = &watchdog_hw->scratch[0];  
  
    // Таблица напряжений: верхняя граница clk_sys -> требуемый VSEL  
    struct VselStep { uint64_t max_hz; vreg_voltage vsel; };  
    const VselStep vsel_table[] = {  
        { 150000000ULL, VREG_VOLTAGE_1_15 },  
        { 200000000ULL, VREG_VOLTAGE_1_20 },  
        { 240000000ULL, VREG_VOLTAGE_1_25 },  
        { 400000000ULL, VREG_VOLTAGE_1_30 },  // максимум VREG, дальше только частотный предел  
    };  
    const size_t vsel_table_size = sizeof(vsel_table) / sizeof(vsel_table[0]);  
  
    struct Candidate { uint32_t fbdiv, p1, p2; uint64_t clk_sys_hz; };  
    static Candidate cand[768];  
    int cand_count = 0;  
  
    // Сбор всех валидных комбинаций выше номинала  
    for (uint32_t fbdiv = 150; fbdiv >= 30; fbdiv--) {  
        uint64_t vco_hz = (uint64_t)fbdiv * VFO_CALIBRATED_XOSC_HZ;  
        if (vco_hz < 750000000ULL || vco_hz > 1600000000ULL) continue;  
  
        for (uint32_t p1 = 2; p1 <= 6; p1++) {  
            for (uint32_t p2 = 1; p2 <= 2; p2++) {  
                uint64_t cs = vco_hz / ((uint64_t)p1 * p2);  
                if (cs <= CLK_START_HZ || cs > CLK_CEIL_HZ) continue;  
                if (cand_count >= (int)(sizeof(cand) / sizeof(cand[0]))) break;  // массив полон — дальше не пишем
                cand[cand_count++] = { fbdiv, p1, p2, cs };  
            }  
        }  
    }  
  
    // Сортировка по возрастанию clk_sys (после заполнения массива!)  
    for (int a = 0; a < cand_count - 1; a++) {  
        for (int b = a + 1; b < cand_count; b++) {  
            if (cand[b].clk_sys_hz < cand[a].clk_sys_hz) {  
                Candidate t = cand[a]; cand[a] = cand[b]; cand[b] = t;  
            }  
        }  
    }  
  
    // Дедупликация одинаковых частот (разные {p1,p2} с одним произведением)  
    int w_pos = 0;  
    for (int r = 0; r < cand_count; r++) {  
        if (r == 0 || cand[r].clk_sys_hz != cand[r - 1].clk_sys_hz) {  
            cand[w_pos++] = cand[r];  
        }  
    }  
    cand_count = w_pos;  
  
    bool max_ok = false;  
    static uint32_t stress_buf[1024];      // 4 КБ SRAM — статический буфер  
  
    // Единый обработчик провала ступени  
    auto fail_step = [&](const char *phase) {  
        Serial.printf("[OCTEST] FAIL (%s) на clk_sys=%lu MHz\n",  
                      phase,  
                      (unsigned long)(pll_overclock.clk_sys_hz / 1000000ULL + 1)); // приближение, ниже точное  
        vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
    };  
  
    for (int i = 0; i < cand_count; i++) {  
  
        // Подбор VSEL по таблице  
        vreg_voltage vsel = VREG_VOLTAGE_1_30;  
        for (size_t k = 0; k < vsel_table_size; k++) {  
            if (cand[i].clk_sys_hz <= vsel_table[k].max_hz) {  
                vsel = vsel_table[k].vsel;  
                break;  
            }  
        }  
  
        PllConfig step_cfg = {  
            cand[i].fbdiv, cand[i].p1, cand[i].p2,  
            cand[i].clk_sys_hz, (uint32_t)vsel, true  
        };  
  
        // Scratch помечает сбойную ступень ДО перехода частоты  
        *scratch = (uint32_t)(cand[i].clk_sys_hz / 1000000ULL);  
  
        // Переход на ступень (напряжение поднимается внутри до смены частоты)  
        vfo_set_clk_sys(step_cfg, (uint32_t)vsel);  
  
        // === КОМПОЗИТНАЯ НАГРУЗКА НА ЯДРО ===  
        volatile uint32_t crc = 0xDEADBEEF;  
        uint32_t t0;  
  
        // Фаза 1: целочисленная арифметика + float — конвейер и FPU-путь  
        {  
            volatile float acc_f = 1.000001f;  
            volatile uint32_t acc_i = 2654435761u;  
            t0 = millis();  
            while (millis() - t0 < 150) {  
                for (int n = 0; n < 2000; n++) {  
                    acc_f = acc_f * acc_f + 0.5f;  
                    acc_i = acc_i * 1664525u + 1013904223u;  
                }  
            }  
            crc ^= (uint32_t)acc_i;  
        }  
  
        // Фаза 2: запись паттерна в SRAM — нагрузка на AHB/SRAM-контроллер  
        t0 = millis();  
        while (millis() - t0 < 100) {  
            for (int w = 0; w < 1024; w++) {  
                stress_buf[w] = crc + (uint32_t)w;  
            }  
            crc++;  
        }  
  
        // Фаза 3: чтение SRAM с CRC-верификацией — ловит тихие искажения AHB  
        {  
            bool sram_ok = true;  
            t0 = millis();  
            while (millis() - t0 < 100 && sram_ok) {  
                for (int w = 0; w < 1024; w++) {  
                    if (stress_buf[w] != crc - 1u + (uint32_t)w) {  
                        sram_ok = false;  
                        break;  
                    }  
                }  
            }  
            if (!sram_ok) {  
                Serial.printf("[OCTEST] FAIL (SRAM CRC) на clk_sys=%lu MHz\n",  
                              (unsigned long)(step_cfg.clk_sys_hz / 1000000ULL));  
                *scratch = (uint32_t)(step_cfg.clk_sys_hz / 1000000ULL);  
                vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
                return;  
            }  
        }  
  
        // Фаза 4: непредсказуемые ветвления — конвейер/предсказание переходов  
        {  
            uint32_t lfsr = crc | 1u;  
            t0 = millis();  
            while (millis() - t0 < 100) {  
                for (int n = 0; n < 4000; n++) {  
                    lfsr ^= lfsr << 13;  
                    lfsr ^= lfsr >> 17;  
                    lfsr ^= lfsr << 5;  
                    if (lfsr & 1)      crc += lfsr;  
                    else               crc ^= lfsr;  
                    if (lfsr & 0x100)  crc = (crc << 3) | (crc >> 29);  
                }  
            }  
        }  
  
        // Фаза 5: аппаратный делитель SIO — инвариант q*den + r == num  
        {  
            bool div_ok = true;  
            uint32_t num = crc | 1u;  
            t0 = millis();  
            while (millis() - t0 < 100 && div_ok) {  
                for (int n = 0; n < 500; n++) {  
                    num = num * 1664525u + 1013904223u;   // LCG-операнды  
                    uint32_t den = (num >> 16) | 1u;      // делитель без нуля  
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
                Serial.printf("[OCTEST] FAIL clk_sys=%6.1f MHz (SRAM CRC)\n",  
                                    (double)(step_cfg.clk_sys_hz / 1000000.0));
                *scratch = (uint32_t)(step_cfg.clk_sys_hz / 1000000ULL);  
                vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
                return;  
            }  
        }  
  
        (void)crc;  
        watchdog_update();  
  
        // Ступень пройдена — печатаем её параметры и запоминаем как потолок  
        uint32_t vco_mhz = step_cfg.fbdiv * (uint32_t)(VFO_CALIBRATED_XOSC_HZ / 1000000ULL); // VCO = fbdiv × 12 МГц  
  
        Serial.printf("[OCTEST] OK  clk_sys=%7.3f MHz  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)  T_CPU=%5.1f C\n",  
                      step_cfg.clk_sys_hz / 1000000.0,  
                      (unsigned long)step_cfg.fbdiv,  
                      (unsigned long)step_cfg.p1,  
                      (unsigned long)step_cfg.p2,  
                      (unsigned long)vco_mhz,  
                      (unsigned)vsel_to_mv((uint32_t)vsel),  
                      step_cfg.clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,  
                      (unsigned)ssi_hw->baudr,  
                      (double)vfo_read_core_temp_c());
        // Фиксируем предел стабильности чипа в долговечную pll_ceiling [Исправлено]
        pll_ceiling = step_cfg;  
        max_ok = true;  
    } // Конец цикла перебора cand_count
  
    *scratch = 0xFFFFFFFFu;  
    vfo_set_clk_sys(pll_nominal, VREG_VOLTAGE_DEFAULT);  
  
    if (max_ok) {  
        Serial.printf("[OCTEST] Потолок стабильности ядра зафиксирован: %6.1f МГц\n",  
                      (double)(pll_ceiling.clk_sys_hz / 1000000.0));
        Serial.println("[OCTEST] Для рабочего разгона рекомендуется запас 10-15% по частоте.");  
    } else {  
        Serial.println("[OCTEST] Ни одна ступень выше номинала не прошла. pll_overclock не изменён.");  
    }  
}



/**  
 * @brief Мгновенная смена активного тона частотной сетки (FSK-переход).  
 *  
 * Перезагружает параметры PIO-делителя и DDS-шаг для движка дизеринга  
 * без разрыва фазы несущей: изменение применяется на следующей итерации  
 * модулятора, а не мгновенной записью в регистр `clkdiv`.  
 *  
 * Два механизма передачи параметров (компайл-тайм):  
 * - `VFO_DITHER_ON_CORE1` — атомарно обновляет @c target_pio_int,  
 *   @c target_pio_frac8 и @c dds_step под аппаратным спинлоком  
 *   @c vfo_spin_lock и взводит флаг @c tone_changed; Core 1  
 *   (@ref vfo_core1_entry) подхватывает новые значения в начале  
 *   следующей итерации и сбрасывает свои аккумуляторы;  
 * - `else` (таймерный режим Core 0) — та же запись переменных под  
 *   `save_and_disable_interrupts`, но с непосредственным сбросом  
 *   аккумуляторов DDS/MASH-2 (@c dds_accumulator, @c dds_accum_m2,  
 *   @c m2_carry_prev), т.к. колбэк @ref vfo_dither_callback читает  
 *   состояние из глобального ОЗУ.  
 *  
 * @param[in] tone_index  Индекс тона в таблице @ref ifkp_tones  
 *                        (0..VFO_IFKP_TONES_COUNT-1). Тон 0 — CW-несущая.  
 *                        Значения вне диапазона игнорируются; повторный  
 *                        вызов с тем же индексом — no-op.  
 *  
 * @note Помечена `__not_in_flash_func` — исполняется из ОЗУ и безопасна  
 *       для вызова из горячего цикла модемов. Сама по себе не включает  
 *       эфир: коммутацию выхода PIO выполняет @ref vfo_operation_set.  
 *       При смене тона в момент передачи на Core 1 сброс аккумуляторов  
 *       происходит на стороне потребителя — фазовый скачок минимален,  
 *       но не нулевой (частотная, а не фазовая непрерывность).  
 *  
 * @see ifkp_tones, vfo_hardware_init(), vfo_core1_entry(),  
 *      vfo_dither_callback(), vfo_operation_set()  
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
 *  
 * Коммутирует выходной пин высокочастотного меандра @c pin_freq_out:  
 * при @p key_down == true пин переводится в режим выхода — PIO-автомат  
 * начинает выдавать меандр (передача в эфир); при @p key_down == false  
 * пин переводится в высокоимпедансный вход — генерация на выходе  
 * мгновенно отсекается без остановки PIO-автомата и дизеринга.  
 *  
 * Реализовано через `pio_sm_set_consecutive_pindirs`: направление пина  
 * переключается синхронно с автоматом PIO, что даёт ключевание без  
 * дребезга и фазовых разрывов посреди периода меандра.  
 *  
 * @param[in] key_down  @c true — ключ нажат (TX, пин = выход);  
 *                      @c false — ключ отпущен (RX/пауза, пин = Hi-Z вход).  
 *  
 * @note Помечена `__not_in_flash_func` — исполняется из ОЗУ, безопасна  
 *       для вызова из горячих циклов модемов (CW, RTTY, IFKP).  
 * @note Закомментированный блок ниже функции (управление пином УМ  
 *       @c pin_amp_act и флагом @c dev_TX_state) — неактивный код:  
 *       планировавшаяся синхронная коммутация питания оконечного каскада  
 *       сейчас не выполняется.  
 *  
 * @see vfo_hardware_init(), vfo_set_tone_instant(), pin_freq_out  
 */
void __not_in_flash_func(vfo_operation_set)(bool key_down) {
    // 1. Управляем направлением пина генератора PIO (высокочастотный меандр)
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, pin_freq_out, 1, key_down);
}



