/**  
 * ============================================================================  
 *  hayes_modem.cpp — Эмулятор Hayes-модема на втором виртуальном COM-порту  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Программная прослойка, которая слушает второй USB CDC порт и имитирует  
 *  поведение Hayes-совместимого модема. Любая терминальная программа или  
 *  софт, умеющий работать с модемом (HyperTerminal, PuTTY, minicom, скрипты),  
 *  может управлять маяком AT-командами и передавать текст в эфир.  
 *  
 *  СОСТОЯНИЯ  
 *  ---------  
 *    КОМАНДНЫЙ РЕЖИМ  — строки "AT..." до S3 (CR) разбираются и исполняются.  
 *    РЕЖИМ ДАННЫХ     — после CONNECT байты считаются данными:  
 *                       петля (ATD0) — немедленное эхо каждого байта;  
 *                       эфир (ATD1..5) — строка копится до CR и уходит в эфир.  
 *    ONLINE COMMAND   — после "+++" соединение сохраняется, модем принимает  
 *                       AT-команды; ATO — возврат в режим данных, ATH — отбой.  
 *  
 *  ESCAPE "+++"  
 *  ------------  
 *  Пауза >= guard (S12 x 20 мс) -> три символа S2 с интервалом < guard ->  
 *  пауза >= guard. Неудачная последовательность возвращается в поток данных.  
 *  S12=0 — без guard-time, S2>127 — escape отключён.  
 *  
 *  ПОДДЕРЖИВАЕМЫЕ КОМАНДЫ  
 *  ----------------------  
 *    AT, A/ (повтор), ATZ, AT&F, AT&W (профиль в ОЗУ, до перезагрузки), AT&V,  
 *    ATE0/1, ATV0/1, ATQ0/1, ATX0..4, ATF0/1 (эхо в режиме данных),  
 *    ATI0..4, ATSn=v, ATSn?, ATH, ATO, ATA, ATD..., AT&D0..3 (реакция на DTR).  
 *    ATL/M/B/N/Y/P/T, AT&C/&K/&S/..., AT\.., AT%.. — принимаются и игнорируются.  
 *  
 *  НАБОР НОМЕРА  
 *  ------------  
 *    ATD[T|P]<режим>[,<частота Гц>][;]   ATDL — повтор последнего номера.  
 *      0 — петля, 1 — CW, 2 — RTTY, 3 — IFKP, 4 — OLIVIA, 5 — THOR.  
 *    Без частоты берётся частота режима из INFO.TXT.  
 *    Пока маяк занят эфиром по расписанию — BUSY (при X<3 — NO CARRIER).  
 *  
 *  СВЯЗИ  
 *  -----  
 *  Передача строки обёрнута так же, как сеансы маяка: is_transmitting=true,  
 *  dev_TX_state, SI_POWER_ON/OFF. Команды STOP/RESTART и запись файлов с ПК  
 *  прерывают передачу и разрывают соединение (NO CARRIER).  
 * ============================================================================  
 */

#include "hayes_modem.h"
#include <Adafruit_TinyUSB.h>
#include "file_manager.h"
#include "si5351_driver.h"
#include "scheduler.h"
#include "cw_modem.h"
#include "rtty_modem.h"
#include "ifkp_modem.h"
#include "olivia_modem.h"
#include "thor_modem.h"
#include "lcd.h"

extern bool soft_restart_flag;
extern bool dev_TX_state;
extern String BCN_VER;
extern String BCN_DAT;

#define HAYES_S_COUNT    32
#define HAYES_CMD_MAX    64
#define HAYES_LINE_MAX   200
#define HAYES_DIAL_MAX   24

enum HayesResult : uint8_t {
  RES_OK         = 0,
  RES_CONNECT    = 1,
  RES_RING       = 2,
  RES_NO_CARRIER = 3,
  RES_ERROR      = 4,
  RES_BUSY       = 7
};

enum HayesAirMode : uint8_t {
  AIR_LOOP   = 0,
  AIR_CW     = 1,
  AIR_RTTY   = 2,
  AIR_IFKP   = 3,
  AIR_OLIVIA = 4,
  AIR_THOR   = 5,
  AIR_COUNT
};

static const char* const AIR_MODE_NAME[AIR_COUNT] = { "LOOP", "CW", "RTTY", "IFKP", "OLIVIA", "THOR" };

struct HayesProfile {
  bool    echo;          // E
  bool    verbose;       // V
  bool    quiet;         // Q
  bool    online_echo;   // F0 -> true
  uint8_t x_level;       // X
  uint8_t dtr_mode;      // &D
  uint8_t s[HAYES_S_COUNT];
};

static const HayesProfile HAYES_FACTORY = {
  true, true, false, true, 4, 2,
  // S0 S1 S2  S3  S4  S5 S6 S7  S8 S9 S10 S11 S12
  {  0, 0, 43, 13, 10, 8, 2, 50, 2, 6, 14, 95, 50 }
};

static Adafruit_USBD_CDC ModemSerial;

static bool         modem_enabled = false;
static HayesProfile cfg;
static HayesProfile stored;

static bool         call_active = false;   // "соединение" установлено
static bool         data_mode   = false;   // true — режим данных, false — командный
static HayesAirMode air_mode    = AIR_LOOP;
static uint32_t     air_freq_hz = 0;

static char   cmd_buf[HAYES_CMD_MAX];
static size_t cmd_len = 0;
static char   last_cmd[HAYES_CMD_MAX] = "";
static char   last_dial[HAYES_DIAL_MAX] = "";

static char   data_buf[HAYES_LINE_MAX + 1];
static size_t data_len   = 0;
static bool   data_ready = false;

static uint8_t  esc_count  = 0;
static uint32_t last_rx_ms = 0;
static bool     prev_dtr   = false;

// ---------------------------------------------------------------------------
// Вывод ответов
// ---------------------------------------------------------------------------
static void modem_crlf() {
  ModemSerial.write(cfg.s[3]);
  ModemSerial.write(cfg.s[4]);
}

static void modem_info(const String& text) {
  if (cfg.quiet) return;
  modem_crlf();
  ModemSerial.print(text);
  modem_crlf();
}

static const char* result_text(uint8_t code) {
  switch (code) {
    case RES_OK:         return "OK";
    case RES_CONNECT:    return "CONNECT";
    case RES_RING:       return "RING";
    case RES_NO_CARRIER: return "NO CARRIER";
    case RES_BUSY:       return "BUSY";
    default:             return "ERROR";
  }
}

static void send_result(uint8_t code) {
  if (code == RES_BUSY && cfg.x_level < 3) code = RES_NO_CARRIER;
  if (cfg.quiet) return;
  if (!cfg.verbose) {
    ModemSerial.print(code);
    ModemSerial.write(cfg.s[3]);
    return;
  }
  modem_crlf();
  ModemSerial.print(result_text(code));
  if (code == RES_CONNECT && cfg.x_level > 0) {
    ModemSerial.print(' ');
    ModemSerial.print(ModemSerial.baud());
  }
  modem_crlf();
}

// ---------------------------------------------------------------------------
// Вспомогательные функции
// ---------------------------------------------------------------------------
static long parse_num(const char*& p) {
  long n = 0;
  while (*p >= '0' && *p <= '9') {
    n = n * 10 + (*p - '0');
    if (n > 99999999L) n = 99999999L;
    p++;
  }
  return n;
}

static uint32_t guard_ms() {
  return (uint32_t)cfg.s[12] * 20UL;
}

static uint32_t default_air_freq(HayesAirMode mode) {
  uint32_t hz = 0;
  switch (mode) {
    case AIR_CW:     hz = strtoul(my_freq_cw_var.c_str(), NULL, 10);     if (hz == 0) hz = 3601000; break;
    case AIR_RTTY:   hz = strtoul(my_rtty_mark_var.c_str(), NULL, 10);   if (hz == 0) hz = 3601585; break;
    case AIR_IFKP:   hz = strtoul(my_freq_ifkp_var.c_str(), NULL, 10);   if (hz == 0) hz = 3601307; break;
    case AIR_OLIVIA: hz = strtoul(my_freq_olivia_var.c_str(), NULL, 10); if (hz == 0) hz = 3601307; break;
    case AIR_THOR:   hz = strtoul(my_freq_thor_var.c_str(), NULL, 10);   if (hz == 0) hz = 3601307; break;
    default: break;
  }
  return hz;
}

static String profile_line(const HayesProfile& p) {
  String s = "E" + String((int)p.echo) + " Q" + String((int)p.quiet) + " V" + String((int)p.verbose) +
             " X" + String((int)p.x_level) + " F" + String(p.online_echo ? 0 : 1) +
             " &D" + String((int)p.dtr_mode);
  return s;
}

static String sreg_line(const HayesProfile& p) {
  String s;
  char buf[12];
  for (uint8_t i = 0; i <= 12; i++) {
    snprintf(buf, sizeof(buf), "S%02u:%03u ", i, p.s[i]);
    s += buf;
  }
  s.trim();
  return s;
}

static String call_line() {
  if (!call_active) return "CALL: NONE";
  String s = "CALL: " + String(AIR_MODE_NAME[air_mode]);
  if (air_mode != AIR_LOOP) s += " " + String(air_freq_hz) + " Hz";
  s += data_mode ? " (DATA)" : " (COMMAND)";
  return s;
}

static void hang_up(const char* reason) {
  if (!call_active) return;
  call_active = false;
  data_mode   = false;
  data_len    = 0;
  data_ready  = false;
  esc_count   = 0;
  Serial.print(F("[Модем] Отбой: "));
  Serial.println(reason);
  LCD_print("                    ", 1, 0);
  LCD_print("MODEM HANG UP", 1, 0);
}

// ---------------------------------------------------------------------------
// Передача накопленной строки в эфир
// ---------------------------------------------------------------------------
static void transmit_data_line() {
  if (is_transmitting || pc_file_written || soft_restart_flag) return;   // эфир занят — ждём

  data_buf[data_len] = '\0';
  String text = String(data_buf);
  data_len   = 0;
  data_ready = false;
  if (text.length() == 0) return;

  is_transmitting = true;
  dev_TX_state = true;
  update_scheduler();
  SI_POWER_ON();

  Serial.print(F("[Модем] ЭФИР "));
  Serial.print(AIR_MODE_NAME[air_mode]);
  Serial.print(F(": "));

  if (!pc_file_written && !soft_restart_flag) {
    switch (air_mode) {
      case AIR_CW:
        prepare_cw_frequency(air_freq_hz);
        send_cw_string(text + " ");
        break;
      case AIR_RTTY: {
        uint32_t shift = strtoul(my_rtty_shift_var.c_str(), NULL, 10);
        if (shift == 0) shift = 170;
        prepare_rtty_frequencies(air_freq_hz, shift, my_rtty_invert_var == "1");
        send_rtty_string(text + "\r\n");
        break;
      }
      case AIR_IFKP:
        prepare_ifkp_frequencies(air_freq_hz);
        send_ifkp_string(text + "\r\n");
        break;
      case AIR_OLIVIA:
        prepare_olivia_frequencies(air_freq_hz);
        olivia_send_string(text + "\r\n");
        break;
      case AIR_THOR:
        prepare_thor_frequencies(air_freq_hz);
        thor_send_string(text + "\r\n");
        break;
      default:
        break;
    }
  }

  bool aborted = pc_file_written || soft_restart_flag;
  is_transmitting = false;
  SI_POWER_OFF();
  dev_TX_state = false;
  update_scheduler();

  if (aborted && call_active) {
    hang_up("передача прервана");
    send_result(RES_NO_CARRIER);
  }
}

// ---------------------------------------------------------------------------
// Режим данных
// ---------------------------------------------------------------------------
static void data_char(uint8_t c) {
  if (air_mode == AIR_LOOP) {  
    ModemSerial.write(c);   // эхо обратно в модемный порт (COM21)  
    Serial.write(c);        // дублируем принятый по петле символ в консольный порт  
    return;  
  }
  if (cfg.online_echo) {
    ModemSerial.write(c);
    if (c == cfg.s[3]) ModemSerial.write(cfg.s[4]);
  }
  if (c == cfg.s[3]) {
    if (data_len > 0) data_ready = true;
  } else if (c == '\n') {
    // LF после CR игнорируется
  } else if (c == cfg.s[5] || c == 0x7F) {
    if (data_len > 0) data_len--;
  } else if (data_len < HAYES_LINE_MAX) {
    data_buf[data_len++] = (char)c;
    if (data_len == HAYES_LINE_MAX) data_ready = true;
  }
}

static void flush_escape_chars() {
  uint8_t n = esc_count;
  esc_count = 0;
  while (n--) data_char(cfg.s[2]);
}

static void enter_online_command() {
  esc_count = 0;
  data_mode = false;
  cmd_len   = 0;
  send_result(RES_OK);
}

static void handle_data_char(uint8_t c, uint32_t now) {
  uint32_t guard = guard_ms();
  if (c == cfg.s[2] && cfg.s[2] <= 127) {
    bool accept = (esc_count == 0) ? (now - last_rx_ms >= guard)
                                   : (esc_count < 3 && (guard == 0 || now - last_rx_ms < guard));
    if (accept) {
      esc_count++;
      if (guard == 0 && esc_count == 3) enter_online_command();
      return;
    }
  }
  flush_escape_chars();
  data_char(c);
}

// ---------------------------------------------------------------------------
// Командный режим
// ---------------------------------------------------------------------------
static void dial(const char* p) {  
  if (call_active) hang_up("повторный набор");   // авто-отбой предыдущего вызова, затем новый набор

  char number[HAYES_DIAL_MAX];
  size_t n = 0;
  bool redial = false;
  bool seen_digit = false;
  for (; *p && *p != ';'; p++) {
    char c = toupper((unsigned char)*p);
    if (c == 'L' && !seen_digit) { redial = true; continue; }
    if ((c >= '0' && c <= '9') || c == ',') {
      if (c != ',') seen_digit = true;
      if (n < sizeof(number) - 1) number[n++] = c;
    }
    // T, P, W, @, !, пробелы, '-', '(', ')' — модификаторы набора, игнорируются
  }
  number[n] = '\0';

  if (redial) {
    if (last_dial[0] == '\0') { send_result(RES_ERROR); return; }
    strncpy(number, last_dial, sizeof(number));
    number[sizeof(number) - 1] = '\0';
    modem_info(String(number));
  }

  const char* q = number;
  while (*q == ',') q++;
  if (*q < '0' || *q > '9') { send_result(RES_ERROR); return; }
  uint8_t mode = *q++ - '0';
  if (mode >= AIR_COUNT) { send_result(RES_NO_CARRIER); return; }

  uint32_t freq = 0;
  for (; *q; q++) {
    if (*q >= '0' && *q <= '9') {
      freq = freq * 10 + (*q - '0');
      if (freq > 300000000UL) { send_result(RES_ERROR); return; }
    }
  }

  if (mode != AIR_LOOP) {
    if (pc_file_written)  { send_result(RES_NO_CARRIER); return; }
    if (is_transmitting)  { send_result(RES_BUSY); return; }
    if (freq == 0) freq = default_air_freq((HayesAirMode)mode);
  }

  strncpy(last_dial, number, sizeof(last_dial));
  last_dial[sizeof(last_dial) - 1] = '\0';

  air_mode    = (HayesAirMode)mode;
  air_freq_hz = freq;
  call_active = true;
  data_mode   = true;
  data_len    = 0;
  data_ready  = false;
  esc_count   = 0;
  last_rx_ms  = millis();

  Serial.print(F("[Модем] CONNECT "));
  Serial.print(AIR_MODE_NAME[air_mode]);
  if (air_mode != AIR_LOOP) { Serial.print(' '); Serial.print(air_freq_hz); Serial.print(F(" Гц")); }
  Serial.println();
  LCD_print("                    ", 1, 0);
  LCD_print(String("MODEM ") + AIR_MODE_NAME[air_mode], 1, 0);

  send_result(RES_CONNECT);
}

static bool show_info(long n) {
  switch (n) {
    case 0: modem_info("RP2040_beacon Hayes emulator"); return true;
    case 1: modem_info("000"); return true;
    case 2: return true;
    case 3: modem_info("RP2040_beacon v" + BCN_VER + " " + BCN_DAT); return true;
    case 4: modem_info("ATD0=LOOP ATD1=CW ATD2=RTTY ATD3=IFKP ATD4=OLIVIA ATD5=THOR [,FREQ_HZ]"); return true;
    default: return false;
  }
}

static void show_profiles() {
  modem_info("ACTIVE PROFILE:\r\n" + profile_line(cfg) + "\r\n" + sreg_line(cfg) +
             "\r\n\r\nSTORED PROFILE:\r\n" + profile_line(stored) + "\r\n" + sreg_line(stored) +
             "\r\n\r\n" + call_line());
}

static void execute_command_line(const char* line) {
  while (*line == ' ') line++;
  if (!((line[0] == 'A' || line[0] == 'a') && (line[1] == 'T' || line[1] == 't'))) return;

  if (line != last_cmd) {
    strncpy(last_cmd, line, sizeof(last_cmd));
    last_cmd[sizeof(last_cmd) - 1] = '\0';
  }

  const char* p = line + 2;
  while (*p) {
    char c = toupper((unsigned char)*p++);
    switch (c) {
      case ' ':
        break;
      case 'E': cfg.echo        = parse_num(p) != 0; break;
      case 'V': cfg.verbose     = parse_num(p) != 0; break;
      case 'Q': cfg.quiet       = parse_num(p) != 0; break;
      case 'F': cfg.online_echo = parse_num(p) == 0; break;
      case 'X': {
        long n = parse_num(p);
        if (n > 4) { send_result(RES_ERROR); return; }
        cfg.x_level = (uint8_t)n;
        break;
      }
      case 'L': case 'M': case 'B': case 'N': case 'Y': case 'P': case 'T':
        parse_num(p);
        break;
      case 'H':
        parse_num(p);
        hang_up("ATH");
        break;
      case 'Z':
        parse_num(p);
        hang_up("ATZ");
        cfg = stored;
        break;
      case 'I':
        if (!show_info(parse_num(p))) { send_result(RES_ERROR); return; }
        break;
      case 'S': {
        long reg = parse_num(p);
        if (reg >= HAYES_S_COUNT) { send_result(RES_ERROR); return; }
        if (*p == '=') {
          p++;
          long v = parse_num(p);
          if (v > 255) { send_result(RES_ERROR); return; }
          cfg.s[reg] = (uint8_t)v;
        } else if (*p == '?') {
          p++;
          char buf[4];
          snprintf(buf, sizeof(buf), "%03u", cfg.s[reg]);
          modem_info(buf);
        }
        break;
      }
      case '&': {
        char sub = toupper((unsigned char)*p);
        if (sub < 'A' || sub > 'Z') { send_result(RES_ERROR); return; }
        p++;
        long n = parse_num(p);
        if      (sub == 'F') cfg = HAYES_FACTORY;
        else if (sub == 'W') stored = cfg;
        else if (sub == 'V') show_profiles();
        else if (sub == 'D') {
          if (n > 3) { send_result(RES_ERROR); return; }
          cfg.dtr_mode = (uint8_t)n;
        }
        break;
      }
      case '\\': case '%':
        if (isalpha((unsigned char)*p)) p++;
        parse_num(p);
        break;
      case 'O':
        parse_num(p);
        if (!call_active) { send_result(RES_NO_CARRIER); return; }
        data_mode  = true;
        esc_count  = 0;
        last_rx_ms = millis();
        send_result(RES_CONNECT);
        return;
      case 'A':
        send_result(call_active ? RES_ERROR : RES_NO_CARRIER);   // входящих вызовов не бывает
        return;
      case 'D':
        dial(p);
        return;
      default:
        send_result(RES_ERROR);
        return;
    }
  }
  send_result(RES_OK);
}

static void handle_command_char(uint8_t c) {
  if (c == cfg.s[3]) {
    if (cfg.echo) ModemSerial.write(c);
    cmd_buf[cmd_len] = '\0';
    cmd_len = 0;
    execute_command_line(cmd_buf);
    return;
  }
  if (c == '\n') return;
  if (c == cfg.s[5] || c == 0x7F) {
    if (cmd_len > 0) {
      cmd_len--;
      if (cfg.echo) ModemSerial.print("\b \b");
    }
    return;
  }
  if (cfg.echo) ModemSerial.write(c);

  if (c == '/' && cmd_len == 1 && (cmd_buf[0] == 'A' || cmd_buf[0] == 'a')) {   // A/ — повтор
    cmd_len = 0;
    if (last_cmd[0] != '\0') execute_command_line(last_cmd);
    return;
  }

  if (cmd_len < HAYES_CMD_MAX - 1) {
    cmd_buf[cmd_len++] = (char)c;
  } else {
    cmd_len = 0;
    send_result(RES_ERROR);
  }
}

static void handle_dtr_drop() {
  cmd_len = 0;
  switch (cfg.dtr_mode) {
    case 1:
      if (data_mode) enter_online_command();
      break;
    case 2:
      hang_up("DTR");
      break;
    case 3:
      hang_up("DTR");
      cfg = stored;
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Публичный API
// ---------------------------------------------------------------------------
void hayes_modem_init() {
  cfg    = HAYES_FACTORY;
  stored = HAYES_FACTORY;

  ModemSerial.setStringDescriptor("Beacon Hayes Modem");
  ModemSerial.begin(115200);
  modem_enabled = (Adafruit_USBD_CDC::getInstanceCount() >= 2);

  // Интерфейс, добавленный после энумерации, появится только после переподключения
  if (modem_enabled && TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

void hayes_modem_poll() {
  if (!modem_enabled) return;

  bool dtr = ModemSerial.dtr();  
  if (prev_dtr && !dtr) handle_dtr_drop();  
  if (!prev_dtr && dtr) {  
    // Терминал только что открыл порт (DTR поднялся): сбрасываем приёмный  
    // буфер и незавершённую командную строку, чтобы спурьёзный байт от  
    // detach()/attach() не "съел" префикс AT первой команды.  
    while (ModemSerial.available() > 0) ModemSerial.read();  
    cmd_len  = 0;  
    esc_count = 0;  
  }  
  prev_dtr = dtr;

  while (ModemSerial.available() > 0) {  
    uint8_t c = (uint8_t)ModemSerial.read();  
    if (data_ready) continue;   // строка уже готова и ждёт эфира — байт отбрасываем, но USB-буфер опустошаем  
    if (data_mode) handle_data_char(c, now);  
    else           handle_command_char(c);  
    last_rx_ms = now;  
  }

  if (data_mode && esc_count > 0 && millis() - last_rx_ms >= guard_ms()) {
    if (esc_count == 3) enter_online_command();
    else                flush_escape_chars();
  }

  if (data_ready && call_active) transmit_data_line();
}

bool hayes_modem_is_online() {
  return call_active;
}

void hayes_modem_print_status() {
  Serial.println(F("=== ЭМУЛЯТОР HAYES-МОДЕМА ==="));
  if (!modem_enabled) {
    Serial.println(F("Второй COM-порт не создан (CFG_TUD_CDC < 2) — модем отключён."));
    return;
  }
  Serial.print(F("Порт: второй USB CDC, DTR="));
  Serial.print(ModemSerial.dtr() ? F("1") : F("0"));
  Serial.print(F(", скорость терминала "));
  Serial.println(ModemSerial.baud());
  Serial.print(F("Профиль: ")); Serial.println(profile_line(cfg));
  Serial.println(sreg_line(cfg));
  Serial.println(call_line());
  Serial.println(F("============================="));
}
