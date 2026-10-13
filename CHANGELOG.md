# История версий RP2040_beacon

## 2.23 — 2026-10-13
- `thor_modem.cpp/.h` переделан в общий IFK/MFSK-движок: `mfsk_prepare(base, step, n)`
  (Si5351 `DATA_MFSK[33][8]` / PIO `vfo_hardware_init`), `mfsk_emit_tone(tone, symbol_us)`.
- Новый режим **MFSK16**: 16 тонов, 15.625 Гц, 64 мс, FEC k=7 + перемежитель 4x10 + код Грея, RSID 57.
- Новый режим **DominoEX 8**: 18 тонов, 15.625 Гц, 128 мс, nibble-varicode, IFK+ (+2), RSID 86.
- Новый режим **FSQ-4.5**: 33 тона, 8.789 Гц, 256 мс, `fsq_varicode`, IFK+ (+1),
  кадр `" \n<CALL>:текст\n "` (без RSID — в fldigi его нет).
- RSID переписан по `rsid.cxx` fldigi (GF16, 15 символов, 16 тонов 10.77 Гц) и вынесен
  в общий `rsid_send(code, center)`; Olivia/Contestia используют его же.
- Интеграция: команды `start mfsk16|dominoex|fsq`, Hayes `ATD7` / `ATD8` / `ATD9`,
  поля `FREQ_MFSK16` / `FREQ_DOMINOEX` / `FREQ_FSQ` в INFO.TXT, режимы `MFSK16` / `DOMINOEX` / `FSQ`
  в расписании (`BeaconMode` 6/7/8).
- README: список режимов, таблица ATD, атрибуция исходных файлов fldigi.

## 2.22 — 2026-10-13
- Правки прошивки и README (подробности не записаны).

## 2.21 — 2026-10-12
- Добавлен эмулятор Hayes-модема на втором COM-порту (`hayes_modem.cpp/.h`).
- Добавлен режим THOR 8/11 (`thor_modem.cpp/.h`).

## 2.20 — 2026-10-10
- Добавлены режимы OLIVIA 8/250 и CONTESTIA 8/250 с RSID (`olivia_modem.cpp/.h`).

## 2.17 — 2026-10-07
- Крупная переработка (15 файлов; подробности не записаны).

## 2.14 — 2026-10-02
- Правки (подробности не записаны).

## 2.13 — 2026-10-02
- Правки (подробности не записаны).

## 2.12 — 2026-10-01
- Добавлен журнал климатических замеров (`climate_log.cpp/.h`).

## 2.11 — 2026-09-30
- Первая версия в репозитории: CW, RTTY, IFKP, синтез Si5351 / PIO-VFO, планировщик,
  USB-диск FAT12, BME280, LCD, светодиод, сборка в GitHub Actions.
