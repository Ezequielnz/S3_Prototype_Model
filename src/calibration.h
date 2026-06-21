#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <Wire.h>

// ═══════════════════════════════════════════════════════════════
//  CONFIGURACIÓN HARDWARE
// ═══════════════════════════════════════════════════════════════
#define MPU_ADDR          0x68
#define PIN_SDA           8
#define PIN_SCL           9
#define PIN_INT           6

// ═══════════════════════════════════════════════════════════════
//  CONFIGURACIÓN SENSOR DE CORRIENTE (SCT013-000)
// ═══════════════════════════════════════════════════════════════
#define PIN_ADC         3
#define MUESTRAS        1000
#define VOLTAJE_REF     3.3
#define ADC_RESOLUCION  4095.0
#define RESISTENCIA     68.0
#define RATIO           2000.0
#define FACTOR_CAL      0.97
#define UMBRAL_RUIDO    0.30

// MPU6050 Registros
#define REG_PWR_MGMT_1    0x6B
#define REG_SMPLRT_DIV    0x19
#define REG_ACCEL_CONFIG  0x1C
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_ACCEL_XOUT_H  0x3B
#define REG_ACCEL_ZOUT_H  0x3F

// Funciones MPU6050
void mpu_write(uint8_t reg, uint8_t val);
bool mpu_read_accel_z(int16_t &z);
bool mpu_read_accel(int16_t &x, int16_t &y, int16_t &z);
void mpu_init();

// Funciones Corriente
float leerCorrienteRMS();

#endif
