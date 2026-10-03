/**  
 * ============================================================================  
 *  vfo_hardware.cpp — Дизеринг-движок программного DDS VFO на автоматах PIO  
 *  Версия 2.14 (Профилирование и ASM-оптимизация Core 1), 2026-10-03  
 * ============================================================================ 
 * программная реализация MASH-2 (Multi-Stage Noise Shaping 2-го порядка
 * выполнен по схеме MASH 1-1 (каскад из двух последовательных дельта-сигма модуляторов 1-го порядка).
 * Конвейер полностью развернут внутри регистров процессора ARM Cortex-M0+ на изолированном ядре Core 1 
 * и работает без обращения к ОЗУ. Благодаря этому частота расчета дизеринга 
 * достигает своего физического потолка — нескольких мегагерц.

/*
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
   └───┬───┘            │ Цифровая комбинация   │     total_correction
       │                │    переносов (NTF)     ├─────► (от -1 до +2)
       ▼                │                        │
   ┌───────┐   carry2   │ Y = C1 + C2 - C2_prev  │
   │ ACC 2 ├───────────►│                        │
   └───────┘            └────────────────────────┘


 *  АРХИТЕКТУРА  
 *  1) PIO гоняет 2-тактный меандр -> f_out = clk_sys / (2 * D).  
 *  2) vfo_find_optimal_pll() перебирает p1(2..6), p2(1..2), fbdiv(30..150),  
 *     отбирает clk_sys (VCO 750..1600 МГц, clk_sys 100..133 МГц) по метрике:  
 *     первично — минимум остатка dds_step (32 бита), вторично — минимум  
 *     pio_frac, третично — максимум clk_sys.  
 *  3) Остаточная дробь делителя доводится DDS/MASH-дизерингом: аккумулятор(ы)  
 *     переполняются с частотой шага, а перенос корректирует clkdiv PIO.  
 *  
 *  МНОГОКРИТЕРИАЛЬНЫЙ АВТОТЮН PLL И ДВУХЪЯДЕРНЫЙ ДИЗЕРИНГ  
 *  -----------------------------------------------------  
 *  1. vfo_find_optimal_pll: Сканирует коэффициенты обратной связи (fbdiv 30..150)  
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

// Физическое размещение массива в RAM. Секция .time_critical гарантирует нахождение в ОЗУ
VfoParameters __attribute__((section(".time_critical.ifkp_tones"))) ifkp_tones[VFO_IFKP_TONES_COUNT];

// ПРАВКА: Доступ к глобальному состоянию усилителя мощности для манипуляции ключом CW
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

/**
 * Быстрый генератор псевдослучайных чисел Xorshift32 в ОЗУ.
 * Добавлен forced inline для полной безопасности конвейера на Core 1
 */
static inline __attribute__((always_inline)) uint32_t __not_in_flash_func(vfo_xorshift32_raw)(uint32_t state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

static inline __attribute__((always_inline)) uint32_t __not_in_flash_func(vfo_xorshift32)() {
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

    while (true) {
        // Опрос флага смены тона (в ОЗУ смотрим только раз за сессию передачи)
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
        int32_t r_bits = (int32_t)(loc_rand_state & 0x0Fu);
        
        // Умножаем на 2 и вычитаем 15. Получаем симметричный ряд нечетных чисел от -15 до +15.
        // Математическое ожидание строго равно 0.0
        step += ((r_bits << 1) - 15); 
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

        // НАДЕЖНАЯ ЗНАКОВАЯ НОРМАЛИЗАЦИЯ: переменные принудительно приведены к int32_t
        int32_t current_frac = l_frac + total_correction;
        int32_t current_int  = l_int;

        // Компилятор гарантированно применит asrs. Если current_frac < 0 (например, -1), из целой части займется 1
        current_int += (current_frac >> 8); 
        current_frac &= 0xFF; // Маска восстановит легальное значение FRAC из отрицательного остатка

        // Единственная за всю итерацию STR-инструкция записи в шину периферии PIO
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


static void detach_peripheral_clock() {
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * 1000000, 48 * 1000000);
}



/**
 * Прецизионный расчет параметров PIO в МИЛЛИГЕРЦАХ (0.001 Гц).
 * Полностью исключает накопление ошибки округления для шага сетки тонов.
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
    
    return params;
}





/**
 * Прецизионный целочисленный расчет параметров в сантигерцах.
 */
 /*
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
*/

/**
 * Расчет аппаратных коэффициентов частоты с Grid Snapping от внешней clk_sys_hz.
 * Изменено: Метрика оценивает исключительно ошибку dds_step.
 */
 /*
static VfoParameters calculate_freq_params(uint64_t clk_sys_hz, unsigned int target_frequency_hz) {
    uint64_t base_target_chz = (uint64_t)target_frequency_hz * 100ULL;

#ifdef VFO_SNAP_TO_GRID
    uint32_t min_dds_metric = 0xFFFFFFFFu; // 32-битный максимум
    VfoParameters best_params = calculate_raw_params_chz(clk_sys_hz, base_target_chz);

    for (int32_t offset_chz = -10; offset_chz <= 10; offset_chz++) {
        uint64_t candidate_chz = (uint64_t)((int64_t)base_target_chz + offset_chz);
        VfoParameters candidate_params = calculate_raw_params_chz(clk_sys_hz, candidate_chz);
        
        // Оценка близости dds_step к краям 32-битной сетки
        uint32_t dstep = candidate_params.dds_step;
        uint32_t dist_to_0 = dstep;
        uint32_t dist_to_max = 0xFFFFFFFFu - dstep;
        uint32_t current_metric = (dist_to_0 < dist_to_max) ? dist_to_0 : dist_to_max;

        if (current_metric < min_dds_metric) {
            min_dds_metric = current_metric;
            best_params = candidate_params;
        }
    }
    return best_params;
#else
    return calculate_raw_params_chz(clk_sys_hz, base_target_chz);
#endif
}
*/




/**
 * Сканирующий матричный алгоритм поиска оптимальной частоты PLL (clk_sys).
 * Направлен на первичную минимизацию 32-битного остатка dds_step.
 * Вторичный критерий — минимизация pio_frac, третичный — выбор максимальной clk_sys.
 */
static PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz) {
    uint64_t crystal_hz = VFO_CALIBRATED_XOSC_HZ;
    uint64_t base_target_chz = (uint64_t)target_frequency_hz * 100ULL;
    
    PllConfig best_pll = { 100, 5, 2, 120000000ULL }; 
#ifdef VFO_CLOCK_133_MHZ
    best_pll = { 133, 6, 2, 133003879ULL };
#endif

    uint32_t min_dds_metric = 0xFFFFFFFFu;       // Предел для 32-битной ошибки DDS
    uint32_t min_frac_metric = 255;              // Предел для ошибки аппаратного делителя PIO

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
                
                // 1. Вычисляем физическое расстояние dds_step до ближайшего края (0 или 2^32)
                uint32_t dist_dds_0 = test_dds_step;
                uint32_t dist_dds_max = 0xFFFFFFFFu - test_dds_step;
                uint32_t current_dds_metric = (dist_dds_0 < dist_dds_max) ? dist_dds_0 : dist_dds_max;

                // ИСПРАВЛЕНИЕ: Защита ближней зоны. Если l_step != 0, но частота переполнения 
                // аккумулятора падает ниже критических 50 кГц, мы искусственно штрафуем эту PLL-комбинацию.
                // 50 кГц при F_s_dither ~5.3 МГц соответствует критической дистанции ~40000 единиц dds_step.
                if (current_dds_metric > 0 && current_dds_metric < 40000u) {
                    // Накладываем жесткий штрафной коэффициент, уводящий метрику из приоритета перебора
                    current_dds_metric += 500000u; 
                }
                
                // 2. Вторичная метрика: Близость pio_frac к целому числу (0 или 256)
                uint32_t dist_frac_0 = test_pio_frac;
                uint32_t dist_frac_max = 256 - test_pio_frac;
                uint32_t current_frac_metric = (dist_frac_0 < dist_frac_max) ? dist_frac_0 : dist_frac_max;

                // Многокритериальный выбор оптимального режима тактирования
                bool is_better_dds  = (current_dds_metric < min_dds_metric);
                bool is_equal_dds   = (current_dds_metric == min_dds_metric);
                bool is_better_frac = (current_frac_metric < min_frac_metric);
                bool is_equal_frac  = (current_frac_metric == min_frac_metric);

                if (is_better_dds || 
                   (is_equal_dds && is_better_frac) ||
                   (is_equal_dds && is_equal_frac && clk_sys_hz > best_pll.clk_sys_hz)) {

                    min_dds_metric = current_dds_metric;
                    min_frac_metric = current_frac_metric;
                    
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
    sm_config_set_set_pins(&c, pin_freq_out, 1); // Прямое использование динамического пина выхода частоты
    
    pio_gpio_init(lo_pio, pin_freq_out); 
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, pin_freq_out, 1, true); 
    
    pio_sm_init(lo_pio, lo_sm, lo_offset, &c);
    pio_sm_set_enabled(lo_pio, lo_sm, true);

    // 1. Рассчитываем точную базовую частоту в сантигерцах
    uint64_t base_target_chz = (uint64_t)base_freq_hz * 100ULL;
    uint64_t snapped_base_chz = base_target_chz;

#ifdef VFO_SNAP_TO_GRID
    // Снаппим ТОЛЬКО базовый тон несущей в окне +-0.1 Гц (+-10 сантигерц)
    uint32_t min_dds_metric = 0xFFFFFFFFu;
    for (int32_t offset_chz = -10; offset_chz <= 10; offset_chz++) {
        uint64_t candidate_chz = (uint64_t)((int64_t)base_target_chz + offset_chz);
        
        // Передаем current_clk_sys_hz и считаем параметры именно для candidate_chz, а не base
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

    // Переводим найденную идеальную опорную частоту обратно в миллигерцы
    uint64_t base_freq_mhz = snapped_base_chz * 10ULL;
    uint64_t step_mhz = (uint64_t)(step_hz * 1000.0);

    // 2. Заполняем таблицу тонов: каждый следующий тон строго равен base + i * step
    // Это гарантирует математически ровную сетку IFKP без рассинхронизации фазы
    for (int i = 0; i < VFO_IFKP_TONES_COUNT; i++) {
        uint64_t tone_freq_mhz = base_freq_mhz + ((uint64_t)i * step_mhz);
        ifkp_tones[i] = calculate_raw_params_mhz(current_clk_sys_hz, tone_freq_mhz);
    }

    vfo_set_tone_instant(0);

    // === ИСТИННЫЙ ДИАГНОСТИЧЕСКИЙ ВЫВОД ПАРАМЕТРОВ БАЗОВОГО ТОНА В SERIAL ===
    VfoParameters real_base_params = ifkp_tones[0]; // Берем параметры CW несущей из рантайм-таблицы
    
/*
    Serial.printf("\n--- VFO Runtime Diagnostics (True Target) ---\n");
    Serial.printf("Target Freq: %u Hz (Grid Freq: %.2f Hz)\n", base_freq_hz, (double)real_base_params.target_freq_chz / 100.0);
    Serial.printf("clk_sys    : %u Hz\n", current_clk_sys_hz);
    Serial.printf("PIO Regs   : INT=%u, FRAC=%u\n", real_base_params.pio_int, real_base_params.pio_frac);
    Serial.printf("DDS Step   : 0x%08X (%u)\n", real_base_params.dds_step, real_base_params.dds_step);

#ifdef VFO_DITHER_FAST
    #ifdef VFO_DITHER_RANDOMIZE
        Serial.printf("Dither Mode: VFO_DITHER_FAST (High-Speed C Loop + RANDOMIZE ON)\n");
    #else
        Serial.printf("Dither Mode: VFO_DITHER_FAST (High-Speed C Loop + RANDOMIZE OFF)\n");
    #endif
#else
    #ifdef VFO_DITHER_RANDOMIZE
        Serial.printf("Dither Mode: Standard C-Version (RANDOMIZE ON)\n");
    #else
        Serial.printf("Dither Mode: Standard C-Version (RANDOMIZE OFF)\n");
    #endif
#endif
    Serial.printf("---------------------------------------------\n");
*/
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

// Синхронное управление ключом PIO и усилителем мощности (УМ) из SET.TXT
void __not_in_flash_func(vfo_operation_set)(bool key_down) {
    // 1. Управляем направлением пина генератора PIO (высокочастотный меандр)
    pio_sm_set_consecutive_pindirs(lo_pio, lo_sm, pin_freq_out, 1, key_down);
}



/*
extern int pin_amp_act; // пин активации УМ из file_manager.cpp

    // 2. Синхронно коммутируем питание оконечного каскада усилителя
    if (key_down) {
        // Нажатие: включаем реле/ключ питания УМ (выставляем HIGH, у вас это GPIO 17)
        gpio_put(pin_amp_act, true);
        dev_TX_state = true;
    } else {
        // Отпускание: мгновенно обесточиваем УМ во избежание перегрева и шума в паузе (LOW)
        gpio_put(pin_amp_act, false);
        dev_TX_state = false;
        
        current_active_tone = VFO_TONE_NONE;
    }
*/