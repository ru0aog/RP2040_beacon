// ax25_modem.cpp — AX.25 300 bd TX: AFSK, NRZI, HDLC, UI-frame
// Разнос тонов: 200 Гц, отклонение от центра +/-100 Гц.

#include "ax25_modem.h"

#include "si5351_driver.h"
#include "file_manager.h"
#include "led_blink.h"
#include "vfo_hardware.h"
#include "thor_modem.h"

#include <hardware/watchdog.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Внешние зависимости
extern bool soft_restart_flag;
extern void si_calculate_freq_bytes_mHz(
    uint64_t freq_mHz,
    uint8_t* out_data
);

// -----------------------------------------------------------------------------
// Параметры AX.25
// -----------------------------------------------------------------------------

#define AX25_SHIFT_HZ       200
#define AX25_HALF_SHIFT     (AX25_SHIFT_HZ / 2)

#define AX25_BAUD           300
#define AX25_BIT_US_NUM     10000ULL
#define AX25_BIT_US_DEN     3ULL

static uint8_t ax25_reg_mark[8];
static uint8_t ax25_reg_space[8];

static bool ax25_inverted = false;
static bool nrzi_tone_mark = true;
static bool tx_aborted = false;

static uint8_t hdlc_ones = 0;

// Абсолютное расписание передачи
static uint32_t sched_start_us = 0;
static uint32_t sched_bit_idx = 0;

// Текущее состояние генератора
static bool tone_initialized = false;
static bool last_tone_mark = true;

// -----------------------------------------------------------------------------
// Проверка запроса остановки
// -----------------------------------------------------------------------------

static inline bool ax25_stop_requested()
{
    if (pc_file_written || soft_restart_flag) {
        tx_aborted = true;
    }

    return tx_aborted;
}

// -----------------------------------------------------------------------------
// Ожидание абсолютного времени начала текущего битового слота
//
// Время слота k:
//   t0 + floor(k * 10000 / 3) микросекунд.
//
// Это предотвращает накопление ошибки округления периода 3333.333 мкс.
// -----------------------------------------------------------------------------

static void ax25_wait_slot()
{
    const uint32_t target =
        sched_start_us +
        (uint32_t)(
            ((uint64_t)sched_bit_idx * AX25_BIT_US_NUM) /
            AX25_BIT_US_DEN
        );

    while ((int32_t)(micros() - target) < 0) {
        if (Serial.available() > 0) {
            soft_restart_flag = true;
        }

        if (ax25_stop_requested()) {
            return;
        }

        delayMicroseconds(10);
        watchdog_update();
    }
}

// -----------------------------------------------------------------------------
// Передача одного битового слота
// -----------------------------------------------------------------------------

static void ax25_bit_slot(bool mark)
{
    ax25_wait_slot();

    if (ax25_stop_requested()) {
        return;
    }

    // Меняем частоту только при изменении логического тона.
    if (!tone_initialized || mark != last_tone_mark) {
        if (device_SI[0]) {
            setFrq_si5351(
                mark ? ax25_reg_mark : ax25_reg_space
            );

            VFO_TX_ON();
        } else {
            // tone 0 = нижний тон MARK;
            // tone 1 = верхний тон SPACE.
            const uint8_t tone =
                mark
                    ? (ax25_inverted ? 1 : 0)
                    : (ax25_inverted ? 0 : 1);

            vfo_set_tone_instant(tone);
        }

        if (mark) {
            ZERO_LED_RED_ON();
        } else {
            ZERO_LED_GREEN_ON();
        }

        last_tone_mark = mark;
        tone_initialized = true;
    }

    ++sched_bit_idx;
}

// -----------------------------------------------------------------------------
// NRZI
//
// Бит 1: сохраняем тон.
// Бит 0: переключаем тон.
// -----------------------------------------------------------------------------

static void ax25_send_bit(uint8_t bit)
{
    if (ax25_stop_requested()) {
        return;
    }

    if (bit == 0) {
        nrzi_tone_mark = !nrzi_tone_mark;
    }

    ax25_bit_slot(nrzi_tone_mark);
}

// -----------------------------------------------------------------------------
// HDLC
// -----------------------------------------------------------------------------

// Передача байта без bit stuffing — используется для флагов.
static void hdlc_put_byte_raw(uint8_t b)
{
    for (int i = 0; i < 8 && !ax25_stop_requested(); ++i) {
        ax25_send_bit((b >> i) & 1u);
    }
}

// Передача байта с bit stuffing.
// После пяти последовательных единиц вставляется ноль.
static void hdlc_put_byte(uint8_t b)
{
    for (int i = 0; i < 8 && !ax25_stop_requested(); ++i) {
        const uint8_t bit = (b >> i) & 1u;

        ax25_send_bit(bit);

        if (bit != 0) {
            if (++hdlc_ones == 5) {
                ax25_send_bit(0);
                hdlc_ones = 0;

                if (ax25_stop_requested()) {
                    return;
                }
            }
        } else {
            hdlc_ones = 0;
        }
    }
}

// -----------------------------------------------------------------------------
// CRC-16/X.25
//
// Reflected polynomial: 0x8408
// Initial value:        0xFFFF
// Final XOR:            0xFFFF
// -----------------------------------------------------------------------------

static uint16_t ax25_fcs_update(uint16_t fcs, uint8_t b)
{
    fcs ^= b;

    for (int i = 0; i < 8; ++i) {
        if (fcs & 1u) {
            fcs = (uint16_t)((fcs >> 1) ^ 0x8408u);
        } else {
            fcs = (uint16_t)(fcs >> 1);
        }
    }

    return fcs;
}

// -----------------------------------------------------------------------------
// AX.25 адрес
//
// 6 символов callsign, дополненных пробелами,
// затем байт SSID.
//
// last=true устанавливает Extension bit последнего адреса.
// -----------------------------------------------------------------------------

static void ax25_put_addr(
    const char* call,
    uint8_t ssid,
    bool last,
    uint16_t& fcs
)
{
    if (call == nullptr) {
        tx_aborted = true;
        return;
    }

    size_t len = strlen(call);

    if (len > 6) {
        len = 6;
    }

    for (int i = 0; i < 6; ++i) {
        char c = (i < (int)len) ? call[i] : ' ';

        c = (char)toupper((unsigned char)c);

        const uint8_t byte =
            (uint8_t)((uint8_t)c << 1);

        fcs = ax25_fcs_update(fcs, byte);
        hdlc_put_byte(byte);

        if (ax25_stop_requested()) {
            return;
        }
    }

    const uint8_t ssid_byte =
        (uint8_t)(
            0x60u |
            ((ssid & 0x0Fu) << 1) |
            (last ? 0x01u : 0x00u)
        );

    fcs = ax25_fcs_update(fcs, ssid_byte);
    hdlc_put_byte(ssid_byte);
}

// -----------------------------------------------------------------------------
// Подготовка частот
// -----------------------------------------------------------------------------

void prepare_ax25_frequencies(uint32_t ax25_freq_hz, bool invert)
{
    ax25_inverted = invert;

    // Общий разнос между тонами — 200 Гц.
    // Каждый тон отклонён от центральной частоты на 100 Гц.
    const uint32_t mark_hz =
        ax25_freq_hz - AX25_HALF_SHIFT;

    const uint32_t space_hz =
        ax25_freq_hz + AX25_HALF_SHIFT;

    if (device_SI[0]) {
        si_calculate_freq_bytes_mHz(
            (uint64_t)mark_hz * 1000ULL,
            ax25_reg_mark
        );

        si_calculate_freq_bytes_mHz(
            (uint64_t)space_hz * 1000ULL,
            ax25_reg_space
        );

        if (debug_flag) {
            Serial.println(
                F("[AX25] Si5351: AFSK shift=200 Hz, 300 baud")
            );

            Serial.print(F("   CENTER: "));
            Serial.println(ax25_freq_hz);

            Serial.print(F("   MARK  : "));
            Serial.println(mark_hz);

            Serial.print(F("   SPACE : "));
            Serial.println(space_hz);
        }
    } else {
        vfo_clk_boost_enter(0);
        vfo_set_narrowband(true);

        // RSID передаётся на центральной частоте AX.25.
        rsid_send(70 /* RSID_AX25 */, (double)ax25_freq_hz);

        // Восстанавливаем настройки сетки AX.25 после RSID.
        vfo_set_narrowband(true);

        // Начальная частота — MARK, разнос — 200 Гц.
        vfo_hardware_init(mark_hz, AX25_SHIFT_HZ);

        if (debug_flag) {
            Serial.println(
                F("[AX25] PIO VFO: AFSK shift=200 Hz, 300 baud")
            );

            Serial.print(F("   CENTER: "));
            Serial.println(ax25_freq_hz);

            Serial.print(F("   MARK  : "));
            Serial.println(mark_hz);

            Serial.print(F("   SPACE : "));
            Serial.println(space_hz);
        }
    }
}

// -----------------------------------------------------------------------------
// Передача AX.25 UI-кадра
// -----------------------------------------------------------------------------

void send_ax25_frame(
    const char* dest_call,
    uint8_t dest_ssid,
    const char* src_call,
    uint8_t src_ssid,
    const uint8_t* info,
    size_t info_len
)
{
    if (!dest_call || !src_call || (!info && info_len > 0)) {
        tx_aborted = true;
        return;
    }

    uint32_t txdelay =
        strtoul(my_ax25_txdelay_var.c_str(), nullptr, 10);

    if (txdelay == 0) {
        txdelay = 30;
    }

    // Начальное состояние передачи.
    tx_aborted = false;
    hdlc_ones = 0;

    nrzi_tone_mark = true;

    tone_initialized = false;
    last_tone_mark = true;

    // Устанавливаем начальный MARK.
    if (device_SI[0]) {
        setFrq_si5351(ax25_reg_mark);
        VFO_TX_ON();
    } else {
        vfo_set_tone_instant(ax25_inverted ? 1 : 0);
    }

    tone_initialized = true;
    last_tone_mark = true;

    // Расписание начинается после установки начального тона.
    sched_start_us = micros();
    sched_bit_idx = 0;

    // Преамбула: последовательность HDLC-флагов.
    for (uint32_t i = 0;
         i < txdelay && !ax25_stop_requested();
         ++i) {
        hdlc_put_byte_raw(0x7E);
    }

    if (ax25_stop_requested()) {
        return;
    }

    // Адреса, Control, PID и информационное поле.
    uint16_t fcs = 0xFFFF;

    ax25_put_addr(dest_call, dest_ssid, false, fcs);
    if (ax25_stop_requested()) return;

    ax25_put_addr(src_call, src_ssid, true, fcs);
    if (ax25_stop_requested()) return;

    // UI frame: Control = 0x03.
    fcs = ax25_fcs_update(fcs, 0x03);
    hdlc_put_byte(0x03);

    // PID = 0xF0: no layer-3 protocol.
    fcs = ax25_fcs_update(fcs, 0xF0);
    hdlc_put_byte(0xF0);

    for (size_t i = 0;
         i < info_len && !ax25_stop_requested();
         ++i) {
        fcs = ax25_fcs_update(fcs, info[i]);
        hdlc_put_byte(info[i]);
    }

    if (ax25_stop_requested()) {
        return;
    }

    // FCS передаётся младшим байтом первым.
    const uint16_t fcs_out = fcs ^ 0xFFFFu;

    hdlc_put_byte((uint8_t)(fcs_out & 0xFFu));
    hdlc_put_byte((uint8_t)((fcs_out >> 8) & 0xFFu));

    // Завершающие флаги, без bit stuffing.
    hdlc_put_byte_raw(0x7E);
    hdlc_put_byte_raw(0x7E);

    if (ax25_stop_requested()) {
        return;
    }

    // Дожидаемся завершения последнего битового слота.
    ax25_wait_slot();
}

// -----------------------------------------------------------------------------
// Передача текстового пакета
// -----------------------------------------------------------------------------

void send_ax25_packet(const String& payload)
{
    Serial.println(payload);

    send_ax25_frame(
        my_ax25_dest_var.length()
            ? my_ax25_dest_var.c_str()
            : "APRS",
        0,
        my_call_variable.c_str(),
        0,
        (const uint8_t*)payload.c_str(),
        payload.length()
    );
}
