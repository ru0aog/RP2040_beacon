#include "bme280.h"
#include <Wire.h>

bool BME_FAIL = true;

// Перечисление для типов датчиков
enum SensorType { TYPE_UNKNOWN, TYPE_BMP280, TYPE_BME280 };
SensorType detectedSensor = TYPE_UNKNOWN; // Храним тип определенного датчика

float  bme_humid;
float  bme_temp;
float  bme_press;

// Структура для хранения калибровочных данных из памяти датчика
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

int32_t t_fine; // Глобальная переменная для компенсации (нужна для расчета давления и влажности)

void I2C_BME_restart() {
  // перезапуск шины Wire на линиях климатического датчика
  Wire.end();
  Wire.setSDA(BME_PIN_SDA);
  Wire.setSCL(BME_PIN_SCL);
  Wire.setClock(400000);
  Wire.begin();
}

// Вспомогательные функции для чтения регистров
uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 1);
  return Wire.read();
}

uint16_t read16(uint8_t reg) {
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 2);
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return (hi << 8) | lo;
}

int16_t readS16(uint8_t reg) {
  return (int16_t)read16(reg);
}

// Чтение калибровочных коэффициентов из датчика
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

// Инициализация и настройка BME280
bool initBME280() {
  // Проверяем Chip ID (для BME280 он равен 0x60. У BMP280 он 0x58)
  uint8_t chipID = read8(0xD0); // Читаем регистр Chip ID

  if (chipID == 0x60) {
    detectedSensor = TYPE_BME280;
  } 
  else if (chipID == 0x58 || chipID == 0x56 || chipID == 0x57) {
    detectedSensor = TYPE_BMP280;
  } 
  else {
    detectedSensor = TYPE_UNKNOWN;
    return false; // Неизвестный чип
  }

  // Сбрасываем датчик
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xE0);
  Wire.write(0xB6);
  Wire.endTransmission();
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
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF2);
    Wire.write(0x01); // передискретизация x1
    Wire.endTransmission();
  }

  // Настройка давления, температуры и режима работы (Регистр 0xF4)
  // Давление x1 (0x01), Температура x1 (0x01), Режим Normal (0x03) -> 0x27
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x27); 
  Wire.endTransmission();

  // Даем датчику время сделать самое первое измерение в режиме Normal
  // По даташиту Bosch первое измерение при оверсэмплинге x1 занимает около 10-15 мс
  delay(20);

  return true;
}


void init_BME() {
  pinMode(BME_POWER_PIN, OUTPUT);
  digitalWrite(BME_POWER_PIN, HIGH);
  delay(100); // Даем чипу DS3231 время на аппаратный старт
  I2C_BME_restart();

  if (!initBME280()) {
    Serial.print("[Система] Датчик не найден! \n");
    BME_FAIL = true;
  }
  else {
    if (detectedSensor == TYPE_BME280) {
      Serial.print("[Система] Датчик BME280 успешно запущен \n");
    } else {
      Serial.print("[Система] Датчик BMP280 успешно запущен \n");
    }
    BME_FAIL = false;
  }
}

// Формулы компенсации из даташита Bosch
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


void get_BME_data() {
  if (BME_FAIL == false) {
    // Начальный регистр данных всегда 0xF7
    I2C_BME_restart();
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    
    // Запрашиваем 8 байт для BME280 или 6 байт для BMP280
    uint8_t bytesToRead = (detectedSensor == TYPE_BME280) ? 8 : 6;
    Wire.requestFrom(BME280_ADDRESS, bytesToRead);

    // Склеиваем сырые данные давления (регистры 0xF7...0xF9)
    uint32_t p_msb  = Wire.read();
    uint32_t p_lsb  = Wire.read();
    uint32_t p_xlsb = Wire.read();
    int32_t adc_P = (p_msb << 12) | (p_lsb << 4) | (p_xlsb >> 4);

    // Склеиваем сырые данные температуры (регистры 0xFA...0xFC)
    uint32_t t_msb  = Wire.read();
    uint32_t t_lsb  = Wire.read();
    uint32_t t_xlsb = Wire.read();
    int32_t adc_T = (t_msb << 12) | (t_lsb << 4) | (t_xlsb >> 4);

    int32_t adc_H = 0;
    // Если датчик BME280, дочитываем оставшиеся 2 байта влажности (регистры 0xFD...0xFE)
    if (detectedSensor == TYPE_BME280) {
      uint32_t h_msb  = Wire.read();
      uint32_t h_lsb  = Wire.read();
      adc_H = (h_msb << 8) | h_lsb;
    }

    // Вычисление реальных физических величин
    float temperature = compensateTemperature(adc_T);
    float pressurePa  = compensatePressure(adc_P);

    // Перевод Паскалей в мм рт. ст.
    float pressureMmHg = pressurePa * 0.00750063755F;
    float humidity;

    // Выводим влажность только если это BME280
    if (detectedSensor == TYPE_BME280) {
      humidity = compensateHumidity(adc_H);
    } else {
      humidity = 0;
    }

    bme_temp = temperature;
    bme_humid = humidity;
    bme_press = pressureMmHg;
  }
}

void BME_read() {
  get_BME_data();
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
  } else {
    Serial.println(" - влажность : нет (Датчик BMP280)");
  }
  Serial.print(" - давление  : ");
  Serial.print(pressureMmHg, 1);  Serial.println(" мм рт. ст.");
}


// Функция для чтения телеметрии (климатические данные)
String get_climate_telemetry() {
  get_BME_data();

  char tele_buf[64]; // Немного увеличили буфер для безопасности
  
  if (detectedSensor == TYPE_BME280) {
    snprintf(tele_buf, sizeof(tele_buf), "T_CL=%.1fC P_CL =%.1fmm H_CL=%.1f%%", bme_temp, bme_press, bme_humid);
  } else {
    // Вариант для BMP280 (без влажности)
    snprintf(tele_buf, sizeof(tele_buf), "T_CL=%.1fC P_CL =%.1fmm", bme_temp, bme_press);
  }
  
  return String(tele_buf);
}