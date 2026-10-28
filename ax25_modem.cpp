// ax25_modem.cpp — AX.25 300 bd TX: AFSK 1200/2200 Гц, NRZI, HDLC, UI-frame  
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
// битовый период 1/300 c: работаем в третях микросекунды (период = 10000/3)  
// -> расписание битов абсолютное: bit_k начинается в t0 + k*10000/3 третей мкс  
  
static uint8_t ax25_reg_mark[8];  
static uint8_t ax25_reg_space[8];  
static bool    ax25_inverted  = false;  
static bool    nrzi_tone_mark = true;  
static bool    tx_aborted     = false;  
static uint8_t hdlc_ones      = 0;  
  
// абсолютное расписание: время начала и номер текущего бита  
static uint32_t sched_start_us;  
static uint32_t sched_bit_idx;  
  
// ---------- прерывание ----------  
static inline bool ax25_stop_requested() {  
  if (pc_file_written || soft_restart_flag) tx_aborted = true;  
  return tx_aborted;  
}  
  
// ---------- физический уровень ----------  
  
// ждём до абсолютного момента начала бита sched_bit_idx  
static void ax25_wait_slot() {  
  // целевое время в мкс: t0 + floor(idx * 10000/3) — делаем в uint64 для точности  
  uint32_t target = sched_start_us +  
                    (uint32_t)(((uint64_t)sched_bit_idx * 10000ULL) / 3ULL);  
  
  while ((int32_t)(micros() - target) < 0) {  
    if (Serial.available() > 0) { soft_restart_flag = true; }  
    if (ax25_stop_requested()) return;  
    delayMicroseconds(10);  
    watchdog_update();  
  }  
}  
  
// один битовый слот: сначала дожидаемся начала слота, потом переключаем тон  
static void ax25_bit_slot(bool mark) {  
  static bool last_tone = true; // для пропуска лишних перенастроек  
  static bool first     = true;  
  
  ax25_wait_slot();  
  if (ax25_stop_requested()) return;  
  
  if (first || mark != last_tone) {  
    if (device_SI[0]) {  
      setFrq_si5351(mark ? ax25_reg_mark : ax25_reg_space);  
      VFO_TX_ON();  
    } else {  
      uint8_t tone = mark ? (ax25_inverted ? 1 : 0)  
                          : (ax25_inverted ? 0 : 1);  
      vfo_set_tone_instant(tone);  
    }  
    if (mark) ZERO_LED_RED_ON(); else ZERO_LED_GREEN_ON();  
    last_tone = mark;  
    first     = false;  
  }  
  sched_bit_idx++;  
}  
  
// NRZI: '1' — тон сохраняется, '0' — переключается  
static void ax25_send_bit(uint8_t bit) {  
  if (ax25_stop_requested()) return;  
  if (!bit) nrzi_tone_mark = !nrzi_tone_mark;  
  ax25_bit_slot(nrzi_tone_mark);  
}  
  
// ---------- HDLC ----------  
  
static void hdlc_put_byte_raw(uint8_t b) {  
  for (int i = 0; i < 8 && !ax25_stop_requested(); i++)  
    ax25_send_bit((b >> i) & 1);  
}  
  
static void hdlc_put_byte(uint8_t b) {  
  for (int i = 0; i < 8 && !ax25_stop_requested(); i++) {  
    uint8_t bit = (b >> i) & 1;  
    ax25_send_bit(bit);  
    if (bit) {  
      if (++hdlc_ones == 5) { ax25_send_bit(0); hdlc_ones = 0; }  
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
  
static void ax25_put_addr(const char* call, uint8_t ssid, bool last, uint16_t& fcs) {  
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
  uint32_t mark_hz  = ax25_freq_hz - AX25_HALF_SHIFT;  
  uint32_t space_hz = ax25_freq_hz + AX25_HALF_SHIFT;  
  
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
    vfo_hardware_init(invert ? space_hz : mark_hz, AX25_SHIFT_HZ);  
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
  uint32_t txdelay = strtoul(my_ax25_txdelay_var.c_str(), NULL, 10);  
  if (txdelay == 0) txdelay = 30;  
  
  // инициализация состояния  
  tx_aborted     = false;  
  hdlc_ones      = 0;  
  nrzi_tone_mark = true;  
  sched_start_us = micros();  
  sched_bit_idx  = 0;  
  { extern void ax25_reset_last_tone(); ax25_reset_last_tone(); } // см. ниже  
  
  for (uint32_t i = 0; i < txdelay && !ax25_stop_requested(); i++)  
    hdlc_put_byte_raw(0x7E);  
  
  uint16_t fcs = 0xFFFF;  
  ax25_put_addr(dest_call, dest_ssid, false, fcs);  
  ax25_put_addr(src_call,  src_ssid,  true,  fcs);  
  fcs = ax25_fcs_update(fcs, 0x03); hdlc_put_byte(0x03);  
  fcs = ax25_fcs_update(fcs, 0xF0); hdlc_put_byte(0xF0);  
  
  for (size_t i = 0; i < info_len && !ax25_stop_requested(); i++) {  
    fcs = ax25_fcs_update(fcs, info[i]);  
    hdlc_put_byte(info[i]);  
  }  
  
  if (ax25_stop_requested()) return;  
  uint16_t fcs_out = fcs ^ 0xFFFF;  
  hdlc_put_byte(fcs_out & 0xFF);  
  hdlc_put_byte((fcs_out >> 8) & 0xFF);  
  hdlc_put_byte_raw(0x7E);  
  hdlc_put_byte_raw(0x7E);  
  
  // дождаться окончания последнего битового слота — тон гасим в вызывающем коде  
}  
  
void send_ax25_packet(const String& payload) {  
  send_ax25_frame(my_ax25_dest_var.length() ? my_ax25_dest_var.c_str() : "APRS", 0,  
                  my_callsign_var.c_str(), 0,  
                  (const uint8_t*)payload.c_str(), payload.length());  
}