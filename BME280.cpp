#include "bme280.h"
#include <Wire.h>

// Перечисление для типов датчиков
enum SensorType { TYPE_UNKNOWN, TYPE_BMP280, TYPE_BME280, TYPE_BMP180 };
SensorType detectedSensor = TYPE_UNKNOWN; // Храним тип определенного датчика

float  bme_humid;
float  bme_temp;
float  bme_press;

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
    // если датчик обнаружен
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    // Настройка пинов и старт шины I2C
    pWire->end();
    pWire->setSDA(device_BM[2]);
    pWire->setSCL(device_BM[3]);
    pWire->begin();
    pWire->setClock(400000);
    //Serial.print("[Система] Рестарт шины датчика давления\n");
  }
}

// Вспомогательные функции для чтения регистров
uint8_t read8(uint8_t reg) {
  uint8_t addr = device_BM[4];
  if (addr == BME280_ADDRESS || addr == BMP180_ADDRESS) {
    // если датчик давления обнаружен
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    // Настройка пинов и старт шины I2C
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
    // если датчик обнаружен
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    // Настройка пинов и старт шины I2C
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

int16_t readS16(uint8_t reg) {
  return (int16_t)read16(reg);
}

int16_t readS16_BE(uint8_t reg) {
  return (int16_t)read16_BE(reg);
}

// Чтение калибровочных коэффициентов из датчика BME280/BMP280
void readCalibrationData() {
  calib.dig_T1 = read16(0x88);
  calib.dig_T2 = readS16(0x8A);
  calib.dig_T3 = readS16(0x8C);

  calib.dig_P1 = read16(0x8E);
  calib.dig_P2 = readS16(0x90);
  calib.dig_P3 = readS16(0x92);
  calib.dig_P4 = readS16(0x94);
  calib.dig_P5 = readS16(0x96);
  calib.dig_P6 = readS16(0x98);
  calib.dig_P7 = readS16(0x9A);
  calib.dig_P8 = readS16(0x9C);
  calib.dig_P9 = readS16(0x9E);

  calib.dig_H1 = read8(0xA1);
  calib.dig_H2 = readS16(0xE1);
  calib.dig_H3 = read8(0xE3);
  
  uint8_t e4 = read8(0xE4);
  uint8_t e5 = read8(0xE5);
  uint8_t e6 = read8(0xE6);
  calib.dig_H4 = (e4 << 4) | (e5 & 0x0F);
  calib.dig_H5 = (e6 << 4) | (e5 >> 4);
  calib.dig_H6 = (int8_t)read8(0xE7);
}

// Чтение калибровочных коэффициентов из датчика BMP180
void readCalibrationDataBMP180() {
  calib180.ac1 = readS16_BE(0xAA);
  calib180.ac2 = readS16_BE(0xAC);
  calib180.ac3 = readS16_BE(0xAE);
  calib180.ac4 = read16_BE(0xB0);
  calib180.ac5 = read16_BE(0xB2);
  calib180.ac6 = read16_BE(0xB4);
  calib180.b1  = readS16_BE(0xB6);
  calib180.b2  = readS16_BE(0xB8);
  calib180.mb  = readS16_BE(0xBA);
  calib180.mc  = readS16_BE(0xBC);
  calib180.md  = readS16_BE(0xBE);
  //Serial.print("[Система] Читаем калибровку BMP180\n");
}

// Инициализация и настройка BME280/BMP280/BMP180
bool initBME280() {
  uint8_t addr = device_BM[4];
  if (addr == BME280_ADDRESS || addr == BMP180_ADDRESS) {
    // если датчик давления обнаружен
    // Выбираем нужный интерфейс Wire
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;

    // Читаем регистр Chip ID
    uint8_t chipID = read8(0xD0); 

    if (chipID == 0x60) {
      detectedSensor = TYPE_BME280;
    }
    else if (chipID == 0x58 || chipID == 0x56 || chipID == 0x57) {
      detectedSensor = TYPE_BMP280;
    }
    else if (chipID == 0x55) {
      detectedSensor = TYPE_BMP180;
    }
    else {
      detectedSensor = TYPE_UNKNOWN;
      return false; // Неизвестный чип
    }
    
    //Serial.print("[Система] Определяем типа датчика\n");
    if (detectedSensor == TYPE_BME280) {
       //Serial.print("[Система] Датчик BME280\n");
    } else if (detectedSensor == TYPE_BMP280) {
       //Serial.print("[Система] Датчик BMP280\n");
    } else if (detectedSensor == TYPE_BMP180) {
       //Serial.print("[Система] Датчик BMP180\n");
    } else if (detectedSensor == TYPE_UNKNOWN) {
       //Serial.print("[Система] Датчик BME/BMP Неизвестный чип \n");
    }

    // Если это старый BMP180, у него инициализация проще
    if (detectedSensor == TYPE_BMP180) {
      readCalibrationDataBMP180();
      return true;
    }

    // Инициализация для BME280 / BMP280
    pWire->beginTransmission(addr);
    pWire->write(0xE0);
    pWire->write(0xB6);
    pWire->endTransmission();
    delay(50);

    // Ждем, пока чип закончит копирование калибровочных данных (регистр status 0xF3, бит 0)
    // Бит 0 (im_update) равен 1, пока данные копируются из NVM памяти чипа
    uint8_t timeout = 100;
    while ((read8(0xF3) & 0x01) && timeout > 0) {
      delay(1);
      timeout--;
    }

    readCalibrationData();

    // Настройка влажности (Регистр 0xF2) — НАСТРАИВАЕМ ТОЛЬКО ДЛЯ BME280
    if (detectedSensor == TYPE_BME280) {
      pWire->beginTransmission(addr);
      pWire->write(0xF2);
      pWire->write(0x01); 
      pWire->endTransmission();
    }

    // Настройка давления, температуры и режима работы (Регистр 0xF4)
    // Давление x1 (0x01), Температура x1 (0x01), Режим Normal (0x03) -> 0x27
    pWire->beginTransmission(addr);
    pWire->write(0xF4);
    pWire->write(0x27); 
    pWire->endTransmission();

    // Даем датчику время сделать самое первое измерение в режиме Normal
    // По даташиту Bosch первое измерение при оверсэмплинге x1 занимает около 10-15 мс
    delay(20);
    return true;
  }
  return false;
}

void init_BME() {
  I2C_BME_restart();

  if (!initBME280()) {
    //Serial.print("[Система] датчик давления, присутствие=");Serial.println(device_BM[0]);    
    //Serial.print("[Система] Wire=");Serial.println(device_BM[1]);
    //Serial.print("[Система] SDA=");Serial.println(device_BM[2]);
    //Serial.print("[Система] SCL=");Serial.println(device_BM[3]);
    //Serial.print("[Система] ADDRESS=");Serial.println(device_BM[4]);

    Serial.print("[Система] ОШИБКА! Внешний датчик BME/BMP не найден! \n");
    device_BM[0] = 0;
  }
  else {
    if (detectedSensor == TYPE_BME280) {
       Serial.print("[Система] Датчик BME280 успешно запущен \n");
    } else if (detectedSensor == TYPE_BMP280) {
       Serial.print("[Система] Датчик BMP280 успешно запущен \n");
    } else if (detectedSensor == TYPE_BMP180) {
       Serial.print("[Система] Датчик BMP180 успешно запущен \n");
    } else if (detectedSensor == TYPE_UNKNOWN) {
       Serial.print("[Система] Датчик BME/BMP Неизвестный чип \n");
    }
    device_BM[0] = 1;
  }
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
  if (device_BM[0] == 1) {
    I2C_BME_restart();
    TwoWire *pWire = (device_BM[1] == 1) ? &Wire1 : &Wire;
    uint8_t addr = device_BM[4];

    // --- Опрос BMP180 (Пошаговый режим) ---
    if (detectedSensor == TYPE_BMP180) {
      // 1. Запрос несформированной температуры (UT)
      pWire->beginTransmission(addr);
      pWire->write(0xF4);
      pWire->write(0x2E);
      pWire->endTransmission();
      delay(5); // Пауза по даташиту на замер температуры

      pWire->beginTransmission(addr);
      pWire->write(0xF6);
      pWire->endTransmission();
      pWire->requestFrom(addr, (uint8_t)2);
      int32_t ut = 0;
      if (pWire->available() >= 2) {
        ut = (pWire->read() << 8) | pWire->read();
      }

      // 2. Запрос несформированного давления (UP), режим OSS=0
      pWire->beginTransmission(addr);
      pWire->write(0xF4);
      pWire->write(0x34);
      pWire->endTransmission();
      delay(5); // Пауза для режима Ultra Low Power

      pWire->beginTransmission(addr);
      pWire->write(0xF6);
      pWire->endTransmission();
      pWire->requestFrom(addr, (uint8_t)2);
      int32_t up = 0;
      if (pWire->available() >= 2) {
        up = (pWire->read() << 8) | pWire->read();
      }

      // 3. Вычисление физических величин
      float temperature = 0;
      float pressurePa = 0;
      computeBMP180(ut, up, temperature, pressurePa);

      bme_temp = temperature;
      bme_humid = 0; // У BMP180 нет датчика влажности
      bme_press = pressurePa * 0.00750063755F; // В мм рт. ст.
      return;
    }

    // --- Опрос BME280 / BMP280 (Потоковый режим) ---
    pWire->beginTransmission(addr);
    pWire->write(0xF7);
    pWire->endTransmission();
    
    uint8_t bytesToRead = (detectedSensor == TYPE_BME280) ? 8 : 6;
    pWire->requestFrom(addr, bytesToRead);

    if (pWire->available() >= bytesToRead) {
      // Склеиваем сырые данные давления (регистры 0xF7...0xF9)
      uint32_t p_msb  = pWire->read();
      uint32_t p_lsb  = pWire->read();
      uint32_t p_xlsb = pWire->read();
      int32_t adc_P = (p_msb << 12) | (p_lsb << 4) | (p_xlsb >> 4);

      // Склеиваем сырые данные температуры (регистры 0xFA...0xFC)
      uint32_t t_msb  = pWire->read();
      uint32_t t_lsb  = pWire->read();
      uint32_t t_xlsb = pWire->read();
      int32_t adc_T = (t_msb << 12) | (t_lsb << 4) | (t_xlsb >> 4);

      int32_t adc_H = 0;
      // Если датчик BME280, дочитываем оставшиеся 2 байта влажности
      if (detectedSensor == TYPE_BME280) {
        uint32_t h_msb  = pWire->read();
        uint32_t h_lsb  = pWire->read();
        adc_H = (h_msb << 8) | h_lsb;
      }
      // [ИСПРАВЛЕНО]: Скобка перенесена сюда. Буфер I2C гарантированно пуст и закрыт!

      // Вычисление реальных физических величин
      float temperature = compensateTemperature(adc_T);
      float pressurePa  = compensatePressure(adc_P);
      float pressureMmHg = pressurePa * 0.00750063755F;
      float humidity = 0;

      if (detectedSensor == TYPE_BME280) {
        humidity = compensateHumidity(adc_H);
      }

      bme_temp = temperature;
      bme_humid = humidity;
      bme_press = pressureMmHg;
    }
  }
}


void BME_read() {
  get_BME_data();
  if (device_BM[0] == 1) {
    float temperature = bme_temp;
    float pressureMmHg = bme_press;
    float humidity = bme_humid;

    // Вывод в Монитор порта
    Serial.print(" - темп.     : ");
    Serial.print(temperature, 1);  Serial.println(" °C");
    
    // Выводим влажность только если это BME280
    if (detectedSensor == TYPE_BME280) {
      Serial.print(" - влажность : ");
      Serial.print(humidity, 1);     Serial.println(" %");
    } else if (detectedSensor == TYPE_BMP280) {
      Serial.println(" - влажность : нет (Датчик BMP280)");
    } else if (detectedSensor == TYPE_BMP180) {
      Serial.println(" - влажность : нет (Датчик BMP180)");
    }
    Serial.print(" - давление  : ");
    Serial.print(pressureMmHg, 1);  Serial.println(" мм рт. ст.");
  }
}


// Функция для чтения телеметрии (климатические данные)
String get_climate_telemetry() {
  get_BME_data();

  char tele_buf[64]; // Зафиксирован размер буфера для предотвращения переполнения
  
  if (detectedSensor == TYPE_BME280) {
    snprintf(tele_buf, sizeof(tele_buf), "T_CL=%.1fC P_CL =%.1fmm H_CL=%.1f%%", bme_temp, bme_press, bme_humid);
  } else {
    // Вариант для BMP280 или BMP180 (без влажности)
    snprintf(tele_buf, sizeof(tele_buf), "T_CL=%.1fC P_CL =%.1fmm", bme_temp, bme_press);
  }
  
  return String(tele_buf);
}