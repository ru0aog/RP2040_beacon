#include "si5351_driver.h"

bool SI_FAIL = true;
uint64_t Xtal_freq  = 25000000;
extern void calculate_freq_bytes_mHz(uint64_t freq_mHz, uint8_t* out_data);

void I2C_SI_restart() {
  // перезапуск шины Wire1 на линиях генератора
  Wire1.end();
  Wire1.setSDA(SI_PIN_SDA);
  Wire1.setSCL(SI_PIN_SCL);
  Wire1.setClock(400000);
  Wire1.begin();
}

bool si5351_write_reg(uint8_t reg, uint8_t data) {
  // запись в регистр Si5351
  // возвращает успешность операции
  if (SI_FAIL == false) {
    // Si5351 присутствует
    I2C_SI_restart();
    for (int i = 0; i < 3; i++) {
      Wire1.beginTransmission(SI5351_I2C_ADDR);
      Wire1.write(reg);
      Wire1.write(data);
      if (Wire1.endTransmission() == 0) {
        return true; 
      }
      if (i < 2) {
        Serial.print("SI5351: I2C error. Retrying ");
        Serial.println(i + 2);
        I2C_SI_restart();
        delay(10);
      }
    }
    return false;
    }
return false;
}

void setFrq_si5351(uint8_t *SI_FREQ_DATA, uint8_t CLK_NO) {
  //быстрая отправка данных частоты
  if (SI_FAIL == false) {
    // Si5351 присутствует
    I2C_SI_restart();
    Wire1.beginTransmission(SI5351_I2C_ADDR);
    if (CLK_NO==0) {Wire1.write(0x2A);}
    if (CLK_NO==1) {Wire1.write(0x32);}
    for (uint8_t i = 0; i < 8; i++) {
      Wire1.write(SI_FREQ_DATA[i]);
    }
    Wire1.endTransmission();
  }
}

void CLK_OFF_si5351(uint8_t CLK_NO) {
  //отключить выход
  if (CLK_NO==0) {si5351_write_reg(0x10, 0x80);} // отключить драйвер CLK0
  if (CLK_NO==1) {si5351_write_reg(0x11, 0x80);} // отключить драйвер CLK1
}

void CLK_ON_si5351(uint8_t CLK_NO) {
  //включить выход
  if (CLK_NO==0) {si5351_write_reg(0x10, 0x0F);} // включить драйвер CLK0
  if (CLK_NO==1) {si5351_write_reg(0x11, 0x0F);} // включить драйвер CLK1
}

void setFr() {
  //установка частоты с расчётом коэффициентов
  static uint32_t frequency11_hz = 1000000;
  uint64_t freq11_mHz = (uint64_t)frequency11_hz * 1000ULL;
  static uint8_t frq_buffer[8]; 
  calculate_freq_bytes_mHz(freq11_mHz, frq_buffer);
  setFrq_si5351(frq_buffer, 0);
}

// Инициализация si5351
void init_si5351_pins() {
  // Настраиваем пины и запускаем шину I2C
  I2C_SI_restart();
  // Сканируем I2C-адрес Si5351 (0x60) для проверки связи
  Wire1.beginTransmission(0x60);
  byte si_status = Wire1.endTransmission();
  if (si_status == 0) {
      SI_FAIL = false;
      Serial.print(F("[Система] Генератор SI-5351 обнаружен на пинах SDA=")); Serial.print(SI_PIN_SDA);Serial.print(", SCL="); Serial.println(SI_PIN_SCL);
      SI_POWER_OFF();
      delay(10);
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

      //03:активировать выходы
      si5351_write_reg(0x03, 0x00);
      Serial.println(" - инициализация CLK0: ОК");
      Serial.println(" - инициализация CLK1: ОК");
      Serial.println("[Система] ГЕН SI-5351: ОК. Генератор успешно запущен и настроен.");
  } else {
      Serial.println(F("[Система] КРИТИЧЕСКАЯ ОШИБКА! ГЕН SI-5351 не найден на шине Wire."));
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
    uint64_t quotient = remainder_mHz / freq_mHz;
    uint64_t remainder2 = remainder_mHz % freq_mHz;
    uint32_t ms_num = (uint32_t)(quotient * ms_den + (remainder2 * ms_den) / freq_mHz);
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

// Функция включения питания si5351
void SI_POWER_ON() {
  I2C_SI_restart();
    //16,17:включить драйверы CLK0,CLK1
    si5351_write_reg(0x10, 0x0F);
    si5351_write_reg(0x11, 0x0F);
    //177:сброс PLL_A,PLL_B
    si5351_write_reg(0xB1, 0xA0);
    //03:активировать выходы
    si5351_write_reg(0x03, 0x00);
    Serial.println("[Питание] Si5351: ВКЛ");
}

// Функция выключения питания si5351
void SI_POWER_OFF() {
  I2C_SI_restart();
    //16-18:снять питание со всех выходов
    si5351_write_reg(0x10, 0x80);
    si5351_write_reg(0x11, 0x80);
    si5351_write_reg(0x12, 0x80);
    //03:отключить все выходы
    si5351_write_reg(0x03, 0xFF);
    Serial.println("[Питание] Si5351: ВЫКЛ");
  Wire1.end();
}
