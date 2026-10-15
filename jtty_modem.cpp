/**  
 * ============================================================================  
 *  jtty_modem.cpp — Реализация передатчика JTTY (По мотивам WSJT-X release/3.3)  
 * ============================================================================  
 *  
 *  Обеспечивает посимвольную блокирующую выдачу 4-FSK кадра JTTY. Вся сетка  
 *  частот готовится через унифицированные процедуры mfsk_*, символы выгоняются  
 *  в эфир по мертвым дедлайнам micros() с постоянным сбросом сторожевого  
 *  таймера и контролем флагов прерывания (pc_file_written, soft_restart_flag).  
 *  
 *  ВНИМАНИЕ (TODO-VERIFY):  
 *    Параметры девиации (JTTY_STEP_HZ), длительности символа (JTTY_SYMBOL_US),  
 *    размерности кадра (JTTY_NDATA) и алгоритм FEC/Interleaver являются  
 *    предварительными. Перед боевым запуском необходима их верификация по  
 *    исходным кодам Fortran-кодека lib/jtty*.f90 оригинального стека WSJT-X.  
 *  
 *  ЛОГИКА ВЫВОДА:  
 *    Несущая частота (carrier) активируется строго в момент старта фрейма  
 *    и безусловно гасится сразу по завершении отправки последнего символа.  
 * ============================================================================  
 */


  
#include "jtty_modem.h"  
#include "vfo_hardware.h"  
#include "thor_modem.h"            // rsid? нет — у JTTY нет RSID; нужен mfsk_prepare прототип  
#include <Arduino.h>  
  
// ===== Внешние флаги проекта (как в rtty_modem.cpp / thor_modem.cpp) =====  
extern uint8_t device_SI[5]; 
extern bool pc_file_written;  
extern bool soft_restart_flag;  
extern bool debug_flag;  
  
// ===== Параметры режима JTTY =====  
// TODO-VERIFY: значения из lib/jtty*.f90 (wsjtx release/3.3)  
#define JTTY_NUMTONES            4                  // 4-FSK  
#define JTTY_STEP_HZ            (2.315f)            // TODO-VERIFY: шаг тонов  
#define JTTY_SYMBOL_US     (432000UL)               // TODO-VERIFY: 1/скорость, мкс  
#define JTTY_NDATA              64                  // TODO-VERIFY: символов полезной нагрузки  
#define JTTY_CENTER_OFFSET_HZ (1.5f * JTTY_STEP_HZ) // центр полосы → нижний тон  
  
static double   JTTY_Base_freq = 0;                 // центр полосы, Гц  
static uint32_t jtty_deadline_us = 0;  
  
// ===== Тайминг (каноническая схема проекта) =====  
  
// Дедлайн первого символа — «сейчас + один символ», как mfsk_sync_deadline()  
static void jtty_sync_deadline(void) {  
    jtty_deadline_us = micros() + JTTY_SYMBOL_US;  
}  
  
// Ждать до конца текущего символа; каждое ожидание сдвигает дедлайн.  
// Возвращает false при запросе остановки.  
static bool jtty_wait_symbol(void) {  
    while ((int32_t)(micros() - jtty_deadline_us) < 0) {  
        if (pc_file_written || soft_restart_flag) return false;  
        watchdog_update();  
        tight_loop_contents();  
    }  
    jtty_deadline_us += JTTY_SYMBOL_US;             // наследуемый дедлайн, без дрейфа  
    return true;  
}  
  
// ===== Сетка тонов =====  
  
// base_hz — ЦЕНТР полосы JTTY (как в prepare_*_frequencies остальных режимов).  
// Сетка строится от нижнего тона: center - (numtones-1)/2 * step.  
void prepare_jtty_frequencies(uint32_t base_hz) {  
    JTTY_Base_freq = base_hz;  
    mfsk_prepare(base_hz - JTTY_CENTER_OFFSET_HZ, JTTY_STEP_HZ, JTTY_NUMTONES);  
}  
  
// ===== Выдача одного символа =====  
  
// tone: 0..JTTY_NUMTONES-1 — индекс в сетке mfsk_prepare  
static bool jtty_emit_tone(uint8_t tone) {  
    mfsk_set_tone(tone);  
    return jtty_wait_symbol();  
}  
  
// ===== Кодирование =====  
  
// TODO-VERIFY: реальное кодирование JTTY:  
//   alphabet → bits → свёрточный код → перемежитель → символы 4-FSK  
//   + вставка sync-вектора.  
// Здесь: временная заглушка — вызывающий код передаёт готовый массив символов  
// тонов (0..3). Портирование jtty_encode()/interleave() из Fortran — следующий шаг.  
  
// ===== Передатчик =====  
  
// symbols: готовый кадр JTTY_NDATA тонов (0..3).  
// Блокирующая функция, как send_rtty_string: при is_transmitting уже  
// должен стоять в вызывающем коде; выход: carrier off.  
bool jtty_send_frame(const uint8_t* symbols, uint8_t nsym) {  
    if (symbols == nullptr || nsym == 0) return false;  
    if (nsym > JTTY_NDATA) nsym = JTTY_NDATA;  
    if (JTTY_Base_freq == 0) return false;          // prepare не вызывали  
  
    if (debug_flag) {  
        Serial.printf("[JTTY] TX: f=%.1f Гц, step=%.3f Гц, %u симв. x %lu мкс\n",  
                      JTTY_Base_freq, (double)JTTY_STEP_HZ,  
                      (unsigned)nsym, (unsigned long)JTTY_SYMBOL_US);  
    }  
  
    // Несущая включается только на символы — перед первым, после последнего off  
    jtty_sync_deadline();  
    mfsk_set_tone(0);  
    mfsk_carrier(true);  
  
    for (uint8_t i = 0; i < nsym; i++) {  
        if (!jtty_emit_tone(symbols[i])) {  
            mfsk_carrier(false);  
            return false;  
        }  
    }  
  
    // Последний символ отзвучал на дедлайне — гасим  
    mfsk_carrier(false);  
    return true;  
}  
  
// Текстовый интерфейс — пока кодирует через заглушку jtty_encode_text.  
// TODO-VERIFY: заменить на порт jtty_encode() из WSJT-X.  
static uint8_t jtty_frame[JTTY_NDATA];  
  
static uint8_t jtty_encode_text(const char* str) {  
    // ЗАГЛУШКА: детерминированное отображение строки в кадр, только для  
    // проверки тракта. Реальный FEC — порт из lib/jtty*.f90.  
    uint8_t n = 0;  
    for (const char* p = str; *p && n < JTTY_NDATA; p++, n++) {  
        jtty_frame[n] = ((uint8_t)*p >> 0) & 3;  
        if (n + 1 < JTTY_NDATA) jtty_frame[++n] = ((uint8_t)*p >> 2) & 3;  
        if (n + 1 < JTTY_NDATA) jtty_frame[++n] = ((uint8_t)*p >> 4) & 3;  
        if (n + 1 < JTTY_NDATA) jtty_frame[++n] = ((uint8_t)*p >> 6) & 3;  
    }  
    return n;  
}  
  
void jtty_send_string(const char* str) {  
    if (str == nullptr) return;  
    uint8_t n = jtty_encode_text(str);  
    jtty_send_frame(jtty_frame, n);  
}