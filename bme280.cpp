/**  
 * ============================================================================  
 *  BME280.cpp — Универсальный регистровый драйвер климатических датчиков  
 *  Версия 2.16 (Исправленная синхронизация BMP280 + AHT20) от 2026-10-01  
 * ============================================================================  
 *  
 *  НАЗНАЧЕНИЕ  
 *  ----------  
 *  Считывает температуру, атмосферное давление и (для BME280) влажность по I2C  
 *  напрямую из регистров чипа, без сторонних библиотек. Полученные данные  
 *  используются для телеметрии маяка. Драйвер универсален и сам определяет  
 *  тип установленного датчика по регистру Chip ID (0xD0).  
 * ============================================================================  
 */

#include "bme280.h"
#include <Wire.h>
#include "file_manager.h" // Обеспечиваем доступ к переменной pin_pwr_bm

// ВНЕШНИЕ ССЫЛКИ: Связываем локальный драйвер с таблицами устройств из главного .ino файла
extern uint8_t device_BM[5];
extern uint8_t device_AH[5];

enum SensorType { TYPE_UNKNOWN, TYPE_BMP280, TYPE_BME280, TYPE_BMP180, TYPE_AHT20 };
SensorType BME_Sensor  = TYPE_UNKNOWN;  // тип датчика Bosch (BMP/BME)
SensorType AHT_Sensor  = TYPE_UNKNOWN;  // тип датчика AHT

// считанные данные AHT
float AHT_temp  = 0.0f;  // температура -/+ 0.3 C
float AHT_humid = 0.0f;  // влажность   -/+ 2.0 %

// считанные данные BME/BMP
float BME_temp  = 0.0f;  // температура -/+ 0.5 C
float BME_press = 0.0f;  // давление    -/+ 0.2 Pa
float BME_humid = 0.0f;  // влажность   -/+ 1.0 %

// Структура для хранения калибровочных данных из памяти датчика BME280/BMP280
struct {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
} calib;

// Структура для хранения калибровочных данных BMP180
struct {
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t b1;
  int16_t b2;
  int16_t mb;
  int16_t mc;
  int16_t md;
} calib180;

int32_t t_fine; // Глобальная переменная для компенсации (нужна для расчета давления и влажности)

void I2C_BME_restart() {
  // перезапуск шины Wire на линиях климатического датчика
  if (device_BM[4] == BME280_ADDRESS || device_BM[4] == BMP180_ADDRESS) {
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    pWire->end();
    pWire->setSDA(device_BM[2]);
    pWire->setSCL(device_BM[3]);
    pWire->begin();
    pWire->setClock(400000);
  }
}

void I2C_AHT_restart() {
  // перезапуск шины Wire на линиях климатического датчика2
  if (device_AH[4] == AHT20_ADDRESS) {
    TwoWire *pWire = (device_AH[1] == 1) ? &Wire1 : &Wire;
    pWire->end();
    pWire->setSDA(device_AH[2]);
    pWire->setSCL(device_AH[3]);
    pWire->begin();
    pWire->setClock(400000);
  }
}

// Вспомогательные функции для чтения регистров
uint8_t read8(uint8_t reg) {
  uint8_t addr = device_BM[4];
  if (addr == BME280_ADDRESS || addr == BMP180_ADDRESS) {
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    pWire->beginTransmission(addr);
    pWire->write(reg);
    pWire->endTransmission();
    pWire->requestFrom(addr, (uint8_t)1);
    if (pWire->available()) return pWire->read();
  }
  return 0;
}

uint16_t read16(uint8_t reg) {
  uint8_t addr = device_BM[4];
  if (addr == BME280_ADDRESS || addr == BMP180_ADDRESS) {
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    pWire->beginTransmission(addr);
    pWire->write(reg);
    pWire->endTransmission();
    pWire->requestFrom(addr, (uint8_t)2);
    if (pWire->available() >= 2) {
      uint8_t lo = pWire->read();
      uint8_t hi = pWire->read();
      return (hi << 8) | lo;
    }
  }
  return 0;
}

// Для BMP180 данные калибрации лежат в формате Big-Endian (MSB первым)
uint16_t read16_BE(uint8_t reg) {
  uint8_t addr = device_BM[4];
  if (addr == BMP180_ADDRESS) {
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    pWire->beginTransmission(addr);
    pWire->write(reg);
    pWire->endTransmission();
    pWire->requestFrom(addr, (uint8_t)2);
    if (pWire->available() >= 2) {
      uint8_t hi = pWire->read();
      uint8_t lo = pWire->read();
      return (hi << 8) | lo;
    }
  }
  return 0;
}

int16_t readS16(uint8_t reg) { return (int16_t)read16(reg); }
int16_t readS16_BE(uint8_t reg) { return (int16_t)read16_BE(reg); }

// Чтение калибровочных коэффициентов из датчика BME280/BMP280
void readCalibrationData() {
  calib.dig_T1 = read16(0x88); calib.dig_T2 = readS16(0x8A); calib.dig_T3 = readS16(0x8C);
  calib.dig_P1 = read16(0x8E); calib.dig_P2 = readS16(0x90); calib.dig_P3 = readS16(0x92);
  calib.dig_P4 = readS16(0x94); calib.dig_P5 = readS16(0x96); calib.dig_P6 = readS16(0x98);
  calib.dig_P7 = readS16(0x9A); calib.dig_P8 = readS16(0x9C); calib.dig_P9 = readS16(0x9E);
  calib.dig_H1 = read8(0xA1);   calib.dig_H2 = readS16(0xE1);  calib.dig_H3 = read8(0xE3);
  
  uint8_t e4 = read8(0xE4); uint8_t e5 = read8(0xE5); uint8_t e6 = read8(0xE6);
  calib.dig_H4 = (e4 << 4) | (e5 & 0x0F);
  calib.dig_H5 = (e6 << 4) | (e5 >> 4);
  calib.dig_H6 = (int8_t)read8(0xE7);
}

// Чтение калибровочных коэффициентов из датчика BMP180
void readCalibrationDataBMP180() {
  calib180.ac1 = readS16_BE(0xAA); calib180.ac2 = readS16_BE(0xAC); calib180.ac3 = readS16_BE(0xAE);
  calib180.ac4 = read16_BE(0xB0);  calib180.ac5 = read16_BE(0xB2);  calib180.ac6 = read16_BE(0xB4);
  calib180.b1  = readS16_BE(0xB6); calib180.b2  = readS16_BE(0xB8); calib180.mb  = readS16_BE(0xBA);
  calib180.mc  = readS16_BE(0xBC); calib180.md  = readS16_BE(0xBE);
}

// Функции AHT20 перенесены вверх файла, чтобы компилятор видел их в блоках init_BME и get_BME_data
void initAHT20() { 
  if (device_AH[0] == 1) {
    I2C_AHT_restart();
    TwoWire *pWire = (device_AH[1] == 1) ? &Wire1 : &Wire;
    uint8_t addr = device_AH[4];
    if (addr == AHT20_ADDRESS) {
      delay(40); 
      pWire->beginTransmission(AHT20_ADDRESS);
      pWire->write(0x71); 
      pWire->endTransmission();
      pWire->requestFrom(AHT20_ADDRESS, (uint8_t)1);
      
      uint8_t status = 0;
      if (pWire->available()) status = pWire->read();
      
      if ((status & 0x08) == 0) {
        pWire->beginTransmission(AHT20_ADDRESS);
        pWire->write(0xBE); pWire->write(0x08); pWire->write(0x00);
        pWire->endTransmission();
        delay(10);
      }
      AHT_Sensor = TYPE_AHT20;
    }
  }
}

void init_AHT() {
  I2C_AHT_restart();
  initAHT20();
  
  if (device_AH[0] == 1) {
    initAHT20();
    if (debug_flag) {
      // вывод сообщения
      Serial.print("[Система] Датчик AHT20 успешно запущен \n");
    }
  }
}


// Инициализация и настройка BME280/BMP280/BMP180
bool initBME280() {
  uint8_t addr = device_BM[4];
  if (pin_pwr_bm != -1) digitalWrite(pin_pwr_bm, HIGH); 
  delay(50); 
  if (addr == BME280_ADDRESS || addr == BMP180_ADDRESS) {
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;

    uint8_t chipID = read8(0xD0); 

    if (chipID == 0x60) {
      BME_Sensor = TYPE_BME280;
    }
    else if (chipID == 0x58 || chipID == 0x56 || chipID == 0x57) {
      BME_Sensor = TYPE_BMP280;
    }
    else if (chipID == 0x55) {
      BME_Sensor = TYPE_BMP180;
    }
    else {
      BME_Sensor = TYPE_UNKNOWN;
      return false; 
    }
    
    if (BME_Sensor == TYPE_BMP180) {
      readCalibrationDataBMP180();
      return true;
    }

    pWire->beginTransmission(addr); pWire->write(0xE0); pWire->write(0xB6); pWire->endTransmission();
    delay(50);

    uint8_t timeout = 100;
    while ((read8(0xF3) & 0x01) && timeout > 0) { delay(1); timeout--; }

    readCalibrationData();

    if (BME_Sensor == TYPE_BME280) {
      pWire->beginTransmission(addr); pWire->write(0xF2); pWire->write(0x01); pWire->endTransmission();
    }

    pWire->beginTransmission(addr); pWire->write(0xF4); pWire->write(0x27); pWire->endTransmission();
    delay(20);
    return true;
  }
  return false;
}

void init_BME() {
  I2C_BME_restart();

  if (!initBME280()) {
    device_BM[0] = 0;
    if (debug_flag) {
      Serial.print("[Система] Внешний датчик BME/BMP не найден! \n");
    }
  } else {
    device_BM[0] = 1;
    if (debug_flag) {
      // вывод сообщения
      if      (BME_Sensor == TYPE_BME280) Serial.print("[Система] Датчик BME280 успешно запущен \n");
      else if (BME_Sensor == TYPE_BMP280) Serial.print("[Система] Датчик BMP280 успешно запущен \n");
      else if (BME_Sensor == TYPE_BMP180) Serial.print("[Система] Датчик BMP180 успешно запущен \n");
    }
  }

  init_AHT();
}




// Формулы компенсации для BME280/BMP280
float compensateTemperature(int32_t adc_T) {
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
  return (float)((t_fine * 5 + 128) >> 8) / 100.0;
}

float compensatePressure(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
  
  if (var1 == 0) return 0; // Защита от деления на ноль
  
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
  return (float)p / 256.0;
}

float compensateHumidity(int32_t adc_H) {
  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) * 
                 (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
                 ((int32_t)calib.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (float)(v_x1_u32r >> 12) / 1024.0;
}

// формулы компенсации вычислений для BMP180
void computeBMP180(int32_t ut, int32_t up, float &temp, float &pressPa) {
  int32_t x1, x2, x3, b3, b5, b6, p;
  uint32_t b4, b7;
  uint8_t oss = 0; // Режим Ultra Low Power (0) для минимизации задержек

  // Расчет температуры
  x1 = (ut - (int32_t)calib180.ac6) * (int32_t)calib180.ac5 >> 15;
  x2 = ((int32_t)calib180.mc << 11) / (x1 + calib180.md);
  b5 = x1 + x2;
  temp = (float)((b5 + 8) >> 4) / 10.0;

  // Расчет давления
  b6 = b5 - 4000;
  x1 = ((int32_t)calib180.b2 * (b6 * b6 >> 12)) >> 11;
  x2 = (int32_t)calib180.ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = ((((int32_t)calib180.ac1 * 4 + x3) << oss) + 2) >> 2;

  x1 = (int32_t)calib180.ac3 * b6 >> 13;
  x2 = ((int32_t)calib180.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (uint32_t)calib180.ac4 * (uint32_t)(x3 + 32768) >> 15;
  b7 = ((uint32_t)up - b3) * (50000 >> oss);

  if (b7 < 0x80000000) {
    p = (b7 * 2) / b4;
  } else {
    p = (b7 / b4) * 2;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p = p + ((x1 + x2 + 3791) >> 4);
  pressPa = (float)p;
}

// Основная функция сбора данных BME280/BMP280/BMP180
void get_BME_data() {
  if (device_BM[0] == 1) { // ИСПРАВЛЕНО: Добавлен индекс [0]
    I2C_BME_restart();
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    uint8_t addr = device_BM[4];

    // --- Опрос BMP180 (Пошаговый режим) ---
    if (BME_Sensor == TYPE_BMP180) {
      // 1. Запрос несформированной температуры (UT)
      pWire->beginTransmission(addr); pWire->write(0xF4); pWire->write(0x2E); pWire->endTransmission();
      delay(5); 

      pWire->beginTransmission(addr); pWire->write(0xF6); pWire->endTransmission();
      pWire->requestFrom(addr, (uint8_t)2);
      int32_t ut = 0; if (pWire->available() >= 2) ut = (pWire->read() << 8) | pWire->read();

      // 2. Запрос несформированного давления (UP), режим OSS=0
      pWire->beginTransmission(addr); pWire->write(0xF4); pWire->write(0x34); pWire->endTransmission();
      delay(5); 

      pWire->beginTransmission(addr); pWire->write(0xF6); pWire->endTransmission();
      pWire->requestFrom(addr, (uint8_t)2);
      int32_t up = 0; if (pWire->available() >= 2) up = (pWire->read() << 8) | pWire->read();

      // 3. Вычисление физических величин
      float temperature = 0; float pressurePa = 0;
      computeBMP180(ut, up, temperature, pressurePa);

      BME_temp = temperature;
      BME_humid = 0; 
      BME_press = pressurePa * 0.00750063755F; 
    }

    // --- Опрос BME280 / BMP280 (Потоковый режим) ---
    pWire->beginTransmission(addr); pWire->write(0xF7); pWire->endTransmission();
    
    uint8_t bytesToRead = (BME_Sensor == TYPE_BME280) ? 8 : 6;
    pWire->requestFrom(addr, bytesToRead);

    if (pWire->available() >= bytesToRead) {
      uint32_t p_msb  = pWire->read(); uint32_t p_lsb  = pWire->read(); uint32_t p_xlsb = pWire->read();
      int32_t adc_P = (p_msb << 12) | (p_lsb << 4) | (p_xlsb >> 4);
      uint32_t t_msb  = pWire->read(); uint32_t t_lsb  = pWire->read(); uint32_t t_xlsb = pWire->read();
      int32_t adc_T = (t_msb << 12) | (t_lsb << 4) | (t_xlsb >> 4);

      int32_t adc_H = 0;
      if (BME_Sensor == TYPE_BME280 && pWire->available() >= 2) {
        uint32_t h_msb  = pWire->read(); uint32_t h_lsb  = pWire->read();
        adc_H = (h_msb << 8) | h_lsb;
      }

      BME_temp = compensateTemperature(adc_T);
      BME_press = compensatePressure(adc_P) * 0.00750063755F;
      BME_humid = (BME_Sensor == TYPE_BME280) ? compensateHumidity(adc_H) : 0;
    }
  }
}

void get_AHT_data() { 
  if (device_AH[0] == 1) {
    I2C_AHT_restart();
    TwoWire *pWire = (device_AH[1] == 1) ? &Wire1 : &Wire;
    
    if (AHT_Sensor == TYPE_AHT20) {
      pWire->beginTransmission(AHT20_ADDRESS);
      pWire->write(0xAC); pWire->write(0x33); pWire->write(0x00);
      pWire->endTransmission();
      delay(80); 

      pWire->requestFrom(AHT20_ADDRESS, (uint8_t)7);
      if (pWire->available() >= 7) {
        uint8_t status = pWire->read(); (void)status;
        
        uint32_t b1 = pWire->read(); uint32_t b2 = pWire->read(); uint32_t b3 = pWire->read();
        uint32_t b4 = pWire->read(); uint32_t b5 = pWire->read(); uint8_t crc = pWire->read(); (void)crc;
        
        uint32_t raw_humidity = (b1 << 12) | (b2 << 4) | (b3 >> 4);
        uint32_t raw_temperature = ((b3 & 0x0F) << 16) | (b4 << 8) | b5;

        AHT_humid = ((float)raw_humidity / 1048576.0f) * 100.0f;
        AHT_temp  = ((float)raw_temperature / 1048576.0f) * 200.0f - 50.0f;
      }
    }
  }
}

// Функция для чтения телеметрии (климатические данные)
String get_climate_telemetry() {
  get_BME_data();
  get_AHT_data();

  char tele_buf[128]; 
  
  snprintf(tele_buf, sizeof(tele_buf), "T_BME=%.1fC P_BME=%.1fmm H_BME=%.1f%% T_AHT=%.1fC H_AHT=%.1f%%", BME_temp, BME_press, BME_humid, AHT_temp, AHT_humid);

  return String(tele_buf);
}





