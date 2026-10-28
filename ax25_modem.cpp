// ax25_modem.cpp — AX.25 300 bd TX: FSK 1200/2200 Гц, NRZI, HDLC, UI-frame  
// FSK-модем AX.25
#include "ax25_modem.h"  
  
// ---- внешние зависимости (как у rtty_modem) ----  
extern bool device_SI[2];  
extern bool soft_restart_flag;  
extern bool pc_file_written;  
extern bool debug_flag;  
extern String my_callsign_var;  
extern String my_ax25_dest_var;  
extern String my_ax25_txdelay_var;  
extern String my_ax25_invert_var;  
  
void watchdog_update();  
void si_calculate_freq_bytes_mHz(uint64_t freq_mHz, uint8_t* out);  
void setFrq_si5351(uint8_t* data);  
void VFO_TX_ON();  
void vfo_clk_boost_enter(uint8_t mode);  
void vfo_set_narrowband(bool nb);  
void vfo_hardware_init(uint32_t base_hz, uint32_t step_hz);  
void vfo_set_tone_instant(uint8_t tone);  
#define ZERO_LED_RED_ON()  
#define ZERO_LED_GREEN_ON()  
  
// ---- параметры ----  
#define AX25_SHIFT_HZ     1000   // разнос AFSK-тонов 1200/2200  
#define AX25_HALF_SHIFT    500  
// битовый период 1/300 c = 10000/3 мкс (3333.33 мкс)  
// расписание абсолютное: бит k начинается в t0 + floor(k * 10000 / 3) мкс  
  
static uint8_t ax25_reg_mark[8];  
static uint8_t ax25_reg_space[8];  
static bool    ax25_inverted  = false;  
static bool    nrzi_tone_mark = true;  
static bool    tx_aborted     = false;  
static uint8_t hdlc_ones      = 0;  
  
// абсолютное расписание: время начала и номер текущего бита  
static uint32_t sched_start_us;  
static uint32_t sched_bit_idx;  
  
// текущий тон генератора — file-scope, сбрасывается в начале каждого кадра  
static bool tone_initialized = false;  
static bool last_tone_mark   = true;  
  
// ---------- прерывание ----------  
static inline bool ax25_stop_requested() {  
  if (pc_file_written || soft_restart_flag) tx_aborted = true;  
  return tx_aborted;  
}  
  
// ---------- физический уровень ----------  
  
// ждём до абсолютного момента начала бита sched_bit_idx  
static void ax25_wait_slot() {  
  // целевое время: t0 + floor(idx * 10000 / 3) мкс — uint64 для точности  
  uint32_t target = sched_start_us +  
                    (uint32_t)(((uint64_t)sched_bit_idx * 10000ULL) / 3ULL);  
  
  while ((int32_t)(micros() - target) < 0) {  
    if (Serial.available() > 0) { soft_restart_flag = true; }  
    if (ax25_stop_requested()) return;  
    delayMicroseconds(10);  
    watchdog_update();  
  }  
}  
  
// один битовый слот: сначала дожидаемся начала слота,  
// потом переключаем генератор — только если тон реально изменился  
static void ax25_bit_slot(bool mark) {  
  ax25_wait_slot();  
  if (ax25_stop_requested()) return;  
  
  if (!tone_initialized || mark != last_tone_mark) {  
    if (device_SI[0]) {  
      setFrq_si5351(mark ? ax25_reg_mark : ax25_reg_space);  
      VFO_TX_ON();  
    } else {  
      // сетка: tone0 = нижний тон (mark), tone1 = верхний (space);  
      // инверсия — только перестановка логических mark/space на индексы  
      uint8_t tone = mark ? (ax25_inverted ? 1 : 0)  
                          : (ax25_inverted ? 0 : 1);  
      vfo_set_tone_instant(tone);  
    }  
    if (mark) ZERO_LED_RED_ON(); else ZERO_LED_GREEN_ON();  
    last_tone_mark   = mark;  
    tone_initialized = true;  
  }  
  sched_bit_idx++;  
}  
  
// NRZI: '1' — тон сохраняется, '0' — тон переключается  
static void ax25_send_bit(uint8_t bit) {  
  if (ax25_stop_requested()) return;  
  if (!bit) nrzi_tone_mark = !nrzi_tone_mark;  
  ax25_bit_slot(nrzi_tone_mark);  
}  
  
// ---------- HDLC ----------  
  
// байт без stuffing (флаги)  
static void hdlc_put_byte_raw(uint8_t b) {  
  for (int i = 0; i < 8 && !ax25_stop_requested(); i++)  
    ax25_send_bit((b >> i) & 1);  
}  
  
// байт со stuffing: вставка '0' после пяти подряд единиц  
static void hdlc_put_byte(uint8_t b) {  
  for (int i = 0; i < 8 && !ax25_stop_requested(); i++) {  
    uint8_t bit = (b >> i) & 1;  
    ax25_send_bit(bit);  
    if (bit) {  
      if (++hdlc_ones == 5) {  
        ax25_send_bit(0);  
        hdlc_ones = 0;  
        if (ax25_stop_requested()) return;  
      }  
    } else {  
      hdlc_ones = 0;  
    }  
  }  
}  
  
// CRC-16/X.25: reflected poly 0x8408, init 0xFFFF, xorout 0xFFFF  
static uint16_t ax25_fcs_update(uint16_t fcs, uint8_t b) {  
  fcs ^= b;  
  for (int i = 0; i < 8; i++)  
    fcs = (fcs & 1) ? (fcs >> 1) ^ 0x8408 : (fcs >> 1);  
  return fcs;  
}  
  
// ---------- адресация (безопасная) ----------  
  
// AX.25 адрес: 6 символов callsign<<1 (padding пробелами) + байт SSID;  
// last=true ставит Extension bit — только у ПОСЛЕДНЕГО адреса цепочки  
static void ax25_put_addr(const char* call, uint8_t ssid, bool last, uint16_t& fcs) {  
  if (!call) { tx_aborted = true; return; }  
  size_t len = strlen(call);  
  if (len > 6) len = 6;            // поле адреса = 6 символов  
  for (int i = 0; i < 6; i++) {  
    char c = (i < (int)len) ? toupper(call[i]) : ' ';  
    uint8_t byte = (uint8_t)(c << 1);  
    fcs = ax25_fcs_update(fcs, byte);  
    hdlc_put_byte(byte);  
    if (ax25_stop_requested()) return;  
  }  
  uint8_t ssid_byte = 0x60 | ((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00);  
  fcs = ax25_fcs_update(fcs, ssid_byte);  
  hdlc_put_byte(ssid_byte);  
}  
  
// ---------- public API ----------  
  
void prepare_ax25_frequencies(uint32_t ax25_freq_hz, bool invert) {  
  ax25_inverted = invert;  
  uint32_t mark_hz  = ax25_freq_hz - AX25_HALF_SHIFT;  // нижний тон  
  uint32_t space_hz = ax25_freq_hz + AX25_HALF_SHIFT;  // верхний тон  
  
  if (device_SI[0]) {  
    si_calculate_freq_bytes_mHz((uint64_t)mark_hz * 1000ULL,  ax25_reg_mark);  
    si_calculate_freq_bytes_mHz((uint64_t)space_hz * 1000ULL, ax25_reg_space);  
    if (debug_flag) {  
      Serial.println(F("[AX25] Si5351: AFSK 1200/2200, 300 bd"));  
      Serial.print(F("   MARK : ")); Serial.println(mark_hz);  
      Serial.print(F("   SPACE: ")); Serial.println(space_hz);  
    }  
  } else {  
    vfo_clk_boost_enter(0);  
    vfo_set_narrowband(true);  
    // vfo_tones[i] = base + i*step — таблица строится ТОЛЬКО вверх от базы,  
    // поэтому база всегда нижний тон (mark_hz), независимо от invert  
    vfo_hardware_init(mark_hz, AX25_SHIFT_HZ);  
    if (debug_flag) {  
      Serial.println(F("[AX25] PIO VFO: AFSK 1200/2200, 300 bd"));  
      Serial.print(F("   MARK : ")); Serial.println(mark_hz);  
      Serial.print(F("   SPACE: ")); Serial.println(space_hz);  
    }  
  }  
}  
  
void send_ax25_frame(const char* dest_call, uint8_t dest_ssid,  
                     const char* src_call,  uint8_t src_ssid,  
                     const uint8_t* info, size_t info_len) {  
  if (!dest_call || !src_call || (!info && info_len > 0)) {  
    tx_aborted = true;  
    return;  
  }  
  
  uint32_t txdelay = strtoul(my_ax25_txdelay_var.c_str(), NULL, 10);  
  if (txdelay == 0) txdelay = 30;  
  
  // инициализация состояния  
  tx_aborted       = false;  
  hdlc_ones        = 0;  
  nrzi_tone_mark   = true;  
  tone_initialized = false;  
  
  // пре-тон MARK до нулевого слота: первый бит не ждёт переключения  
  if (device_SI[0]) {  
    setFrq_si5351(ax25_reg_mark);  
    VFO_TX_ON();  
  } else {  
    vfo_set_tone_instant(ax25_inverted ? 1 : 0);  
  }  
  tone_initialized = true;  
  last_tone_mark   = true;  
  
  // расписание стартует ПОСЛЕ установки начального тона  
  sched_start_us   = micros();  
  sched_bit_idx    = 0;  
  
  // преамбула — флаги (TX delay)  
  for (uint32_t i = 0; i < txdelay && !ax25_stop_requested(); i++)  
    hdlc_put_byte_raw(0x7E);  
  
  // заголовок + payload, попутно считаем FCS  
  uint16_t fcs = 0xFFFF;  
  ax25_put_addr(dest_call, dest_ssid, false, fcs);  // Extension=0  
  ax25_put_addr(src_call,  src_ssid,  true,  fcs);  // Extension=1 (последний адрес)  
  fcs = ax25_fcs_update(fcs, 0x03); hdlc_put_byte(0x03);  // control: UI  
  fcs = ax25_fcs_update(fcs, 0xF0); hdlc_put_byte(0xF0);  // PID: no L3  
  
  for (size_t i = 0; i < info_len && !ax25_stop_requested(); i++) {  
    fcs = ax25_fcs_update(fcs, info[i]);  
    hdlc_put_byte(info[i]);  
  }  
  
  if (ax25_stop_requested()) return;  
  
  uint16_t fcs_out = fcs ^ 0xFFFF;  
  hdlc_put_byte(fcs_out & 0xFF);        // FCS: младший байт первым  
  hdlc_put_byte((fcs_out >> 8) & 0xFF);  
  
  // завершающие флаги  
  hdlc_put_byte_raw(0x7E);  
  hdlc_put_byte_raw(0x7E);  
  if (ax25_stop_requested()) return;  
  
  // последний бит завершающего флага должен прозвучать полный слот:  
  // sched_bit_idx указывает на слот ПОСЛЕ последнего переданного бита  
  ax25_wait_slot();  
}  
  
void send_ax25_packet(const String& payload) {  
  send_ax25_frame(my_ax25_dest_var.length() ? my_ax25_dest_var.c_str() : "APRS", 0,  
                  my_callsign_var.c_str(), 0,  
                  (const uint8_t*)payload.c_str(), payload.length());  
}
