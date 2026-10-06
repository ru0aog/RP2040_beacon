/**  
 * ============================================================================  
 *  si5351_driver.cpp — Прямой I2C-драйвер ВЧ-синтезатора Si5351  
 *  Версия 2.12 от 2026-10-01, автор RU0AOG  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Обеспечивает прямое низкоуровневое управление внешним ВЧ-синтезатором частоты  
 *  Si5351 по шине I2C (Wire или Wire1) без использования сторонних библиотек.  
 *  Формирует чистую несущую на выходах CLK0 (основной сигнал) и CLK1.    
 *  
 *  АЛГОРИТМЫ И НАДЕЖНОСТЬ ШИНЫ  
 *  ----------------------------  
 *  - Отказоустойчивость: Функция si5351_write_reg выполняет до 3 попыток записи  
 *    регистра. При сбое ACK автоматически вызывается I2C_SI_restart(), который  
 *    переинициализирует аппаратный I2C-контроллер RP2040, предотвращая зависание.  
 *  - Быстрая манипуляция (setFrq_si5351): Позволяет за одну I2C-транзакцию  
 *    записать предрассчитанный 8-байтный пакет параметров в регистры MultiSynth  
 *    (0x2A для CLK0, 0x32 для CLK1), что критично для RTTY и IFKP мод.  
 *  - Дробный синтез (calculate_freq_bytes_mHz): Переводит частоту из миллигерц  
 *    в параметры делителей P1, P2, P3 со статическим знаменателем MultiSynth (1048575).  
 *    Опорная частота PLL_A зафиксирована на Xtal_freq * 36 (900 МГц).  
 *  
 *  АРХИТЕКТУРНЫЙ ОБХОД (Резервирование)  
 *  ------------------------------------  
 *  Если при старте сканер не обнаружил чип Si5351 на шине (флаг SI_FAIL = true),  
 *  методы управления выходом VFO_TX_ON / VFO_TX_OFF прозрачно перенаправляют  
 *  команды на ключевание программного DDS VFO (`vfo_operation_set`).  
 * ============================================================================  
 *  
 *  ИНИЦИАЛИЗАЦИЯ (init_si5351)  
 *  ---------------------------  
 *  Сканирует адрес 0x60; при успехе снимает SI_FAIL, подаёт питание (SI_POWER_ON),  
 *  настраивает кварц (нагрузочная ёмкость 10 пФ, spread-spectrum выкл),  
 *  программирует PLL_A (MSNA, VCO 900 МГц) и делители MultiSynth MS0/MS1 для  
 *  CLK0/CLK1, сбрасывает PLL и активирует выходы. При отсутствии чипа выводит  
 *  предупреждение и переключает систему на внутренний DDS-генератор (GPIO 28).  
 *  
 *  РАСЧЁТ ЧАСТОТЫ (calculate_freq_bytes_mHz)  
 *  -----------------------------------------  
 *  Переводит целевую частоту (в миллигерцах) в 8-байтный пакет регистров  
 *  MultiSynth (P1/P2/P3, дробный делитель со знаменателем 1048575) для быстрой  
 *  отправки через setFrq_si5351(). PLL зафиксирована на Xtal_freq * 36.  
 *  ВАЖНО: та же функция используется модулятором RTTY для расчёта частот  
 *  MARK/SPACE (объявлена extern).  
 *  
 *  УПРАВЛЕНИЕ ВЫХОДОМ  
 *  ------------------  
 *  VFO_TX_ON / VFO_TX_OFF — вкл/выкл драйвера CLK0/CLK1 (регистры  
 *  0x10/0x11); при отсутствии чипа вызывают vfo_operation_set(true/false).  
 *  SI_POWER_ON / SI_POWER_OFF — подача/снятие питания с выходов и сброс PLL.  
 *  format_freq() — форматирование частоты для вывода (МГц,кГц.Гц).  
 * ============================================================================  
 */

#include "si5351_driver.h"
#include "vfo_hardware.h"
#include "file_manager.h" // Обеспечиваем доступ к переменной pin_pwr_si

extern bool dev_TX_state; 
extern int  pin_pwr_si;

bool SI_FAIL = true;
uint64_t Xtal_freq  = 25000000;
extern void calculate_freq_bytes_mHz(uint64_t freq_mHz, uint8_t* out_data);

void I2C_SI_restart() {
  if (device_SI[0]) {
    // перезапуск шины Wire на линиях генератора
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_SI[1] == 1) ? &Wire1 : &Wire;
    // Настройка пинов и старт шины I2C
    pWire->end();
    pWire->setSDA(device_SI[2]);
    pWire->setSCL(device_SI[3]);
    pWire->setClock(400000);
    pWire->begin();
  }
}

bool si5351_write_reg(uint8_t reg, uint8_t data) {
  // запись в регистр Si5351
  // Выбираем нужный интерфейс Wire
  TwoWire *pWire = (device_SI[1] == 1) ? &Wire1 : &Wire;
  // возвращает успешность операции
  if (device_SI[0]) {
    // Si5351 присутствует
    I2C_SI_restart();
    for (int i = 0; i < 3; i++) {
      pWire->beginTransmission(SI5351_I2C_ADDR);
      pWire->write(reg);
      pWire->write(data);
      if (pWire->endTransmission() == 0) {
        return true; 
      }
      if (i < 2) {
        Serial.print("SI5351: ошибка шины I2C. Перезапуск ");
        Serial.println(i + 2);
        I2C_SI_restart();
        delay(10);
      }
    }
    return false;
    }
return false;
}

void setFrq_si5351(uint8_t *SI_FREQ_DATA) {
  //быстрая отправка данных частоты
  if (device_SI[0]) {
    // Si5351 присутствует
    I2C_SI_restart();
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_SI[1] == 1) ? &Wire1 : &Wire;
    pWire->beginTransmission(SI5351_I2C_ADDR);
    if (si5351_clk_tx_out==0) {pWire->write(0x2A);}
    if (si5351_clk_tx_out==1) {pWire->write(0x32);}
    if (si5351_clk_tx_out==2) {pWire->write(0x3A);}
    for (uint8_t i = 0; i < 8; i++) {
      pWire->write(SI_FREQ_DATA[i]);
    }
    pWire->endTransmission();
  }
}









/**  
 * @brief Инициализация внешнего ВЧ-синтезатора Si5351 по шине I2C.  
 *  
 * Выполняет полную конфигурацию чипа без сторонних библиотек:  
 * -# <b>Шина.</b> @ref I2C_SI_restart настраивает Wire/Wire1 на пины из  
 *    дескриптора @c device_SI (400 кГц).  
 * -# <b>Обнаружение.</b> Сканирует адрес 0x60; при ACK снимается флаг  
 *    @c SI_FAIL — дальнейшие вызовы @ref VFO_TX_ON/@ref VFO_TX_OFF будут  
 *    работать с чипом, иначе команды прозрачно перенаправляются на  
 *    внутренний DDS-генератор через @ref vfo_operation_set.  
 * -# <b>Питание и безопасный сброс.</b> @ref SI_POWER_ON (питание через  
 *    @c pin_pwr_si, пауза 50 мс, запуск драйверов и PLL), затем гашение  
 *    всех выходов (рег. 0x03 = 0xFF) и обесточивание драйверов  
 *    (рег. 0x10-0x12 = 0x80) — исключает выбросы на этапе настройки.  
 * -# <b>Базовая конфигурация.</b> Источник тактирования PLL — кварц  
 *    (рег. 0x0F = 0x00); нагрузочная ёмкость кварца 10 пФ  
 *    (рег. 0xB7 = 0xC0); размытие спектра выключено (рег. 0x95 = 0x00).  
 * -# <b>PLL_A = 900 МГц.</b> MultiSynth NA программируется  
 *    целочисленным делителем ×36 (MSNA_P1=0x1000, P2=0, P3=1,  
 *    рег. 0x1A-0x21), затем сброс PLL (рег. 0xB1 = 0xA0).  
 * -# <b>Делители выходов.</b> MultiSynth MS0/MS1 для CLK0/CLK1  
 *    программируются жёстко зашитыми дробными делителями (рег.  
 *    0x2A-0x31 и 0x32-0x39) — стартовые частоты по умолчанию;  
 *    далее частоту меняет @ref setFrq_si5351.  
 * -# <b>Активация.</b> Драйверы CLK0/CLK1 включаются в дробном режиме  
 *    от PLL_A, ток 8 мА (рег. 0x10/0x11 = 0x0F); выход TX сразу  
 *    глушится @ref VFO_TX_OFF, затем все выходы активируются  
 *    (рег. 0x03 = 0x00).  
 *  
 * При отсутствии чипа на шине выводится предупреждение в Serial,  
 * @c SI_FAIL остаётся true, и система работает через внутренний  
 * PIO-DDS на пине @c VFO_OUTPUT_PIN.  
 *  
 * @note Вызывается из `setup()` один раз. Все записи регистров идут  
 *       через @ref si5351_write_reg — до 3 попыток с автоперезапуском  
 *       шины при сбое ACK.  
 * @note Активный выходной канал TX задаётся глобальной  
 *       @c si5351_clk_tx_out (из SET.TXT, тег SI5351_CLK_OUT) — она  
 *       выбирает регистры 0x10/0x11/0x12 в @ref VFO_TX_ON/OFF и базовый  
 *       регистр MultiSynth в @ref setFrq_si5351; здесь же все три  
 *       канала конфигурируются независимо от неё.  
 *  
 * @see I2C_SI_restart(), si5351_write_reg(), SI_POWER_ON(),  
 *      VFO_TX_ON(), VFO_TX_OFF(), setFrq_si5351(),  
 *      calculate_freq_bytes_mHz()  
 */
void init_si5351() {
  // Настраиваем пины и запускаем шину I2C
  I2C_SI_restart();
  if (device_SI[0]){
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_SI[1] == 1) ? &Wire1 : &Wire;
    // Сканируем I2C-адрес Si5351 (0x60) для проверки связи
    pWire->beginTransmission(0x60);
    byte si_status = pWire->endTransmission();
    if (si_status == 0) {
      SI_FAIL = false;
      //Serial.print(F("[Система] Генератор SI-5351 обнаружен на пинах SDA=")); Serial.print(SI_PIN_SDA);Serial.print(", SCL="); Serial.println(SI_PIN_SCL);
      //SI_POWER_OFF();
      //delay(10);
      SI_POWER_ON();
      //03:отключить все выходы
      si5351_write_reg(0x03, 0xFF);
      //16-18:снять питание со всех выходов
      si5351_write_reg(0x10, 0x80);
      si5351_write_reg(0x11, 0x80);
      si5351_write_reg(0x12, 0x80);
      //15:установить источник тактирования PLL_A,PLL_B
      si5351_write_reg(0x0F, 0x00); //кварц
      //183:установить нагрузочную ёмкость кварца
      si5351_write_reg(0xB7, 0xC0); //10 пФ
      //149:отключить размытие спектра
      si5351_write_reg(0x95, 0x00);
      //26-33:установить Multisynth NA (PLL_A = 900 МГц)
      //MSNA_P1 = 0x001000 = 4096
      //MSNA_P2 = 0x00000
      //MSNA_P3 = 0x00001
      si5351_write_reg(0x1A, 0x00); //MSNA_P3[15:8]
      si5351_write_reg(0x1B, 0x01); //MSNA_P3[7:0]
      si5351_write_reg(0x1C, 0x00); //MSNA_P1[17:16]
      si5351_write_reg(0x1D, 0x10); //MSNA_P1[15:8]
      si5351_write_reg(0x1E, 0x00); //MSNA_P1[7:0]
      si5351_write_reg(0x1F, 0x00); //MSNA_P3[19:16], MSNA_P2[19:16]
      si5351_write_reg(0x20, 0x00); //MSNA_P2[15:18]
      si5351_write_reg(0x21, 0x00); //MSNA_P2[7:0]
      //177:сброс PLL_A,PLL_B
      si5351_write_reg(0xB1, 0xA0);

      //42-49:установить Multisynth0 CLK0
      //MS0_P1 = 0x07BAB
      //MS0_P2 = 0x44408
      //MS0_P3 = 0xDA8E8
      //R0_DIV = 0x0
      //MS0_DIVBY4 = 0x0
      si5351_write_reg(0x2A, 0xA8); //MS0_P3[15:8]
      si5351_write_reg(0x2B, 0xE8); //MS0_P3[7:0]
      si5351_write_reg(0x2C, 0x00); //R0_DIV[2:0], MS0_DIVBY4[1:0], MS0_P1[17:16]
      si5351_write_reg(0x2D, 0x7B); //MS0_P1[15:8]
      si5351_write_reg(0x2E, 0xAB); //MS0_P1[7:0]
      si5351_write_reg(0x2F, 0xD4); //MS0_P3[19:16], MS0_P2[19:16]
      si5351_write_reg(0x30, 0x44); //MS0_P2[15:8]
      si5351_write_reg(0x31, 0x08); //MS0_P2[7:0]
      //16:включить CLK0 в дробном режиме от MultiSynth 0, источник PLL_A, нагрузка 8 мА.
      si5351_write_reg(0x10, 0x0F);

      //50-58:установить Multisynth1 CLK1
      //MS1_P1 = 0x06E83
      //MS1_P2 = 0x43BC0
      //MS1_P3 = 0xF4240
      //R1_DIV = 0x0
      //MS1_DIVBY4 = 0x0
      si5351_write_reg(0x32, 0x42); //MS1_P3[15:8]
      si5351_write_reg(0x33, 0x40); //MS1_P3[7:0]
      si5351_write_reg(0x34, 0x00); //R1_DIV[2:0], MS1_DIVBY4[1:0], MS1_P1[17:16]
      si5351_write_reg(0x35, 0x6E); //MS1_P1[15:8]
      si5351_write_reg(0x36, 0x83); //MS1_P1[7:0]
      si5351_write_reg(0x37, 0xF4); //MS1_P3[19:16], MS1_P2[19:16]
      si5351_write_reg(0x38, 0x3B); //MS1_P2[15:8]
      si5351_write_reg(0x39, 0xC0); //MS1_P2[7:0]
      //17:включить CLK1 в дробном режиме от MultiSynth 1, источник PLL_A, нагрузка 8 мА.
      si5351_write_reg(0x11, 0x0F);
      VFO_TX_OFF();            // отключить выход частоты TX
      //03:активировать выходы
      si5351_write_reg(0x03, 0x00);
      //Serial.println(" - инит CLK0 : ОК");
      //Serial.println(" - инит CLK1 : ОК");
      //Serial.println("[Система] ГЕН SI-5351: ОК. Генератор успешно запущен и настроен.");
    }
  }

  if (!device_SI[0]) {
      Serial.println(F("[Система] КРИТИЧЕСКАЯ ОШИБКА! ГЕН SI-5351 не найден на шине Wire."));
      Serial.print(F("[Система] ВНИМАНИЕ! Будет использован внутренний DDS-генератор на пине ")); Serial.println(VFO_OUTPUT_PIN);
      SI_FAIL = true;
    }
}

// Функция расчета регистров si5351
void calculate_freq_bytes_mHz(uint64_t freq_mHz, uint8_t* out_data) {
  if (freq_mHz == 0) return;
    const uint32_t pll_multiplier = 36;
    uint64_t f_pll_mHz = (uint64_t)Xtal_freq * pll_multiplier * 1000ULL;
    uint32_t divider_int = (uint32_t)(f_pll_mHz / freq_mHz);
    uint64_t remainder_mHz = f_pll_mHz % freq_mHz;
    uint32_t ms_den = 1048575;
    uint32_t ms_num = (uint32_t)((remainder_mHz * ms_den) / freq_mHz);
    uint32_t p1 = 128 * divider_int + ((128 * ms_num) / ms_den) - 512;
    uint32_t p2 = 128 * ms_num - ms_den * ((128 * ms_num) / ms_den);
    uint32_t p3 = ms_den;
    out_data[0] = (p3 >> 8) & 0xFF;
    out_data[1] = p3 & 0xFF;
    out_data[2] = (p1 >> 16) & 0x03;
    out_data[3] = (p1 >> 8) & 0xFF;
    out_data[4] = p1 & 0xFF;
    out_data[5] = ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F);
    out_data[6] = (p2 >> 8) & 0xFF;
    out_data[7] = p2 & 0xFF;
}

// Функция разделения частоты пробелами
String format_freq(uint32_t FREQ) {
  uint32_t mhz  = FREQ / 1000000;          
  uint32_t khz  = (FREQ % 1000000) / 1000; 
  uint32_t hz   = FREQ % 1000;             

  char freq_buf[32];
  snprintf(freq_buf, sizeof(freq_buf), "%lu,%03lu.%03lu", mhz, khz, hz);
  
  return String(freq_buf);
}




void VFO_TX_ON() {
  // включает выход CLK_X
  // включает питание драйвера
  // источник тактов - MultiSynth 0
  // ток 8 мА
  if (device_SI[0]) {
    if (si5351_clk_tx_out==0) {si5351_write_reg(0x10, 0x0F);} // включить драйвер CLK0
    if (si5351_clk_tx_out==1) {si5351_write_reg(0x11, 0x0F);} // включить драйвер CLK1
    if (si5351_clk_tx_out==2) {si5351_write_reg(0x12, 0x0F);} // включить драйвер CLK2
  }
  else {
    vfo_operation_set(true);    // запустить генерацию программного VFO
    // потом заменить на загрузку инструкций ядра1
  }
}

void VFO_TX_OFF() {
  // отключает выход CLK_X
  // выключить питание драйвера
  if (device_SI[0]) {
    if (si5351_clk_tx_out==0) {si5351_write_reg(0x10, 0x80);} // отключить драйвер CLK0
    if (si5351_clk_tx_out==1) {si5351_write_reg(0x11, 0x80);} // отключить драйвер CLK1
    if (si5351_clk_tx_out==2) {si5351_write_reg(0x12, 0x80);} // отключить драйвер CLK2
  }
  else {
    vfo_operation_set(false);   // отключить генерацию программного VFO
  }
}


// Функция включения питания VFO
void SI_POWER_ON() {
  
  if (device_SI[0]) {
    // запуск питания
    if (pin_pwr_si != -1) digitalWrite(pin_pwr_si, HIGH); // Включаем генератор
    delay(50); // Небольшая пауза на стабилизацию питания перед инициализацией чипов
    // перезапуск шины Wire на линиях генератора
    I2C_SI_restart();
    //16,17:включить драйверы CLK0,CLK1
    si5351_write_reg(0x10, 0x0F);
    si5351_write_reg(0x11, 0x0F);
    //177:сброс PLL_A,PLL_B
    si5351_write_reg(0xB1, 0xA0);
    //03:активировать выходы
    si5351_write_reg(0x03, 0x00);
    if (debug_flag) {
      Serial.println("[Питание] Si5351: ВКЛ");
    }
    dev_TX_state = true;
  }
  else {
    if (debug_flag) {
      Serial.println("[Система] ВНИМАНИЕ! Отсутствует модуль Si5351");
      Serial.print("[Питание] Запускаем внутренний DDS-генератор RP2040 на пине "); Serial.println(VFO_OUTPUT_PIN);
    }
  }
}

// Функция выключения питания si5351
void SI_POWER_OFF() {
  extern int pin_pwr_si;
  if (device_SI[0]) {
    // перезапуск шины Wire на линиях генератора
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_SI[1] == 1) ? &Wire1 : &Wire;
    I2C_SI_restart();
    //16-18:снять питание со всех выходов
    si5351_write_reg(0x10, 0x80);
    si5351_write_reg(0x11, 0x80);
    si5351_write_reg(0x12, 0x80);
    //03:отключить все выходы
    si5351_write_reg(0x03, 0xFF);
    pWire->end();
    if (debug_flag) {
      Serial.println("[Питание] Si5351: ВЫКЛ");
    }
    // запуск питания
    if (pin_pwr_si != -1) digitalWrite(pin_pwr_si, LOW); // Выключаем генератор
  }
  else {
    if (debug_flag) {
      Serial.print("[Питание] DDS-генератор RP2040 на пине "); Serial.print(VFO_OUTPUT_PIN); Serial.println(" остановлен.");
    }
    vfo_operation_set(false);   // отключить генерацию программного VFO
    vfo_clk_boost_exit();   // выход из режима разгона
  }
}




