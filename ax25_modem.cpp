// ax25_modem.cpp — AX.25 300 bd TX: Bell 202 (1200/2200), NRZI, HDLC, UI-frame  
#include "ax25_modem.h"  
  
// ---- внешние зависимости (те же, что у rtty_modem) ----  
extern bool device_SI[2];  
extern bool soft_restart_flag;  
extern bool pc_file_written;  
extern bool debug_flag;  
extern String my_callsign_var;      // позывной маяка  
extern String my_ax25_dest_var;     // destination, default "APRS"  
extern String my_ax25_txdelay_var;  // число флагов TX-delay, default "30"  
extern String my_ax25_invert_var;   // "0"/"1"  
  
void watchdog_update();  
void si_calculate_freq_bytes_mHz(uint64_t freq_mHz, uint8_t* out);  
void setFrq_si5351(uint8_t* data);  
void VFO_TX_ON();  
void vfo_clk_boost_enter(uint8_t mode);  
void vfo_set_narrowband(bool nb);  
void vfo_hardware_init(uint32_t base_hz, uint32_t step_hz);  
void vfo_set_tone_instant(uint8_t tone);  
#define ZERO_LED_RED_ON()    // уже определены в проекте  
#define ZERO_LED_GREEN_ON()  
  
// ---- параметры модуляции ----  
#define AX25_BAUD            300  
#define AX25_SHIFT_HZ        1000   // mark 1200 / space 2200  
#define AX25_HALF_SHIFT      500  
// 1/300 c = 3333.333 мкс — накопитель дробной части: 3333, 3333, 3334, ...  
static uint32_t ax25_frac_acc = 0;  // трети микросекунды: период = 10000/3  
  
static uint8_t ax25_reg_mark[8];  
static uint8_t ax25_reg_space[8];  
static bool    ax25_inverted = false;  
static bool    nrzi_tone_mark = true; // текущий тон: стартуем с MARK  
  
// ---------- физический уровень ----------  
  
static void ax25_wait_bit_time() {  
  // дробный период 3333.(3) мкс через аккумулятор третей мкс  
  ax25_frac_acc += 10000;  
  uint32_t bit_us = ax25_frac_acc / 3;  
  ax25_frac_acc -= bit_us * 3;  
  
  uint32_t start = micros();  
  while (micros() - start < bit_us) {  
    if (Serial.available() > 0) { soft_restart_flag = true; return; }  
    delayMicroseconds(10);  
    watchdog_update();  
  }  
}  
  
static void ax25_tone(bool mark) {  
  if (pc_file_written || soft_restart_flag) return;  
  
  if (device_SI[0]) {  
    setFrq_si5351(mark ? ax25_reg_mark : ax25_reg_space);  
    VFO_TX_ON();  
  } else {  
    // сетка: tone0 = нижний тон (mark при норме), tone1 = верхний (space)  
    uint8_t tone = mark ? (ax25_inverted ? 1 : 0)  
                        : (ax25_inverted ? 0 : 1);  
    vfo_set_tone_instant(tone);  
  }  
  
  if (mark) ZERO_LED_RED_ON(); else ZERO_LED_GREEN_ON();  
  ax25_wait_bit_time();  
}  
  
// NRZI: '1' — тон сохраняется, '0' — тон переключается  
static void ax25_send_bit(uint8_t bit) {  
  if (!bit) nrzi_tone_mark = !nrzi_tone_mark;  
  ax25_tone(nrzi_tone_mark);  
}  
  
// ---------- HDLC ----------  
  
static uint8_t hdlc_ones = 0;  
  
// байт без stuffing (флаги)  
static void hdlc_put_byte_raw(uint8_t b) {  
  for (int i = 0; i < 8; i++) ax25_send_bit((b >> i) & 1); // LSB-first  
}  
  
// байт со stuffing: вставка '0' после пяти подряд единиц  
static void hdlc_put_byte(uint8_t b) {  
  for (int i = 0; i < 8; i++) {  
    uint8_t bit = (b >> i) & 1;  
    ax25_send_bit(bit);  
    if (bit) {  
      if (++hdlc_ones == 5) { ax25_send_bit(0); hdlc_ones = 0; }  
    } else {  
      hdlc_ones = 0;  
    }  
  }  
  if (soft_restart_flag || pc_file_written) return;  
}  
  
// FCS: CRC-16-CCITT reflected, init 0xFFFF, poly 0x8408, xorout 0xFFFF  
static uint16_t ax25_fcs_update(uint16_t fcs, uint8_t b) {  
  for (int i = 0; i < 8; i++) {  
    if (((fcs ^ b) & 1)) fcs = (fcs >> 1) ^ 0x8408;  
    else                 fcs >>= 1;  
    b >>= 1;  
  }  
  return fcs;  
}  
  
// AX.25 адрес: 6 символов callsign<<1 (padding пробелами) + байт SSID  
static void ax25_put_addr(const char* call, uint8_t ssid, bool last, uint16_t& fcs) {  
  for (int i = 0; i < 6; i++) {  
    char c = call[i] ? call[i] : ' ';  
    c = toupper(c);  
    fcs = ax25_fcs_update(fcs, (uint8_t)(c << 1));  
    hdlc_put_byte((uint8_t)(c << 1));  
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
      Serial.println(F("[AX25] Сетка Bell 202 для Si5351 готова."));  
      Serial.print(F("   MARK : ")); Serial.print(mark_hz);  Serial.println(F(" Hz"));  
      Serial.print(F("   SPACE: ")); Serial.print(space_hz); Serial.println(F(" Hz"));  
    }  
  } else {  
    vfo_clk_boost_enter(0);  
    vfo_set_narrowband(true);  
    uint32_t base_hz = invert ? space_hz : mark_hz;  
    vfo_hardware_init(base_hz, AX25_SHIFT_HZ);  
    if (debug_flag) {  
      Serial.println(F("[AX25] Программный VFO: Bell 202, 300 bd."));  
      Serial.print(F("   MARK : ")); Serial.print(mark_hz);  Serial.println(F(" Hz"));  
      Serial.print(F("   SPACE: ")); Serial.print(space_hz); Serial.println(F(" Hz"));  
    }  
  }  
}  
  
void send_ax25_frame(const char* dest_call, uint8_t dest_ssid,  
                     const char* src_call,  uint8_t src_ssid,  
                     const uint8_t* info, size_t info_len) {  
  uint32_t txdelay = strtoul(my_ax25_txdelay_var.c_str(), NULL, 10);  
  if (txdelay == 0) txdelay = 30;  
  
  nrzi_tone_mark = true;   // NRZI стартует с mark  
  ax25_frac_acc = 0;  
  hdlc_ones = 0;  
  
  // преамбула — флаги (TX delay)  
  for (uint32_t i = 0; i < txdelay; i++) {  
    hdlc_put_byte_raw(0x7E);  
    if (soft_restart_flag || pc_file_written) return;  
  }  
  
  // заголовок + FCS  
  uint16_t fcs = 0xFFFF;  
  ax25_put_addr(dest_call, dest_ssid, false, fcs);  
  ax25_put_addr(src_call,  src_ssid,  true,  fcs);  
  fcs = ax25_fcs_update(fcs, 0x03); hdlc_put_byte(0x03); // control: UI  
  fcs = ax25_fcs_update(fcs, 0xF0); hdlc_put_byte(0xF0); // PID: no L3  
  
  for (size_t i = 0; i < info_len; i++) {  
    fcs = ax25_fcs_update(fcs, info[i]);  
    hdlc_put_byte(info[i]);  
    if (soft_restart_flag || pc_file_written) return;  
  }  
  
  uint16_t fcs_out = fcs ^ 0xFFFF;  
  hdlc_put_byte(fcs_out & 0xFF);        // FCS, младший байт первым  
  hdlc_put_byte((fcs_out >> 8) & 0xFF);  
  
  // завершающие флаги  
  hdlc_put_byte_raw(0x7E);  
  hdlc_put_byte_raw(0x7E);  
}  
  
void send_ax25_packet(const String& payload) {  
  send_ax25_frame(my_ax25_dest_var.length() ? my_ax25_dest_var.c_str() : "APRS", 0,  
                  my_callsign_var.c_str(), 0,  
                  (const uint8_t*)payload.c_str(), payload.length());  
}  
  
void ax25_self_test() {  
  // эталонный кадр: APRS-0 > CALL-0, UI, "TEST" — сверить FCS с приёмником  
  uint16_t fcs = 0xFFFF;  
  const char* d = "APRS";  
  const char* s = "TEST";  
  for (int i = 0; i < 6; i++) fcs = ax25_fcs_update(fcs, (uint8_t)((d[i]?d[i]:' ') << 1));  
  fcs = ax25_fcs_update(fcs, 0x60);  
  for (int i = 0; i < 6; i++) fcs = ax25_fcs_update(fcs, (uint8_t)(toupper(s[i]?s[i]:' ') << 1));  
  fcs = ax25_fcs_update(fcs, 0x61);  
  fcs = ax25_fcs_update(fcs, 0x03);  
  fcs = ax25_fcs_update(fcs, 0xF0);  
  for (const char* p = "TEST"; *p; p++) fcs = ax25_fcs_update(fcs, *p);  
  Serial.print(F("[AX25-SELFTEST] FCS=0x"));  
  Serial.println(fcs ^ 0xFFFF, HEX); // проверка: на приёме residue должен быть 0xF0B8  
}