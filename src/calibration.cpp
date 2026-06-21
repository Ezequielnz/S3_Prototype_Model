#include "calibration.h"

// MPU implementations
void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}

bool mpu_read_accel_z(int16_t &z) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_ZOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true) != 2) return false;
    z = (Wire.read() << 8) | Wire.read();
    return true;
}

bool mpu_read_accel(int16_t &x, int16_t &y, int16_t &z) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true) != 6) return false;
    x = (Wire.read() << 8) | Wire.read();
    y = (Wire.read() << 8) | Wire.read();
    z = (Wire.read() << 8) | Wire.read();
    return true;
}

void mpu_init() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(1000000);  // 1 MHz — Fast Mode+
    Wire.setTimeOut(10);     // Timeout 10ms anti-cuelgues por ruido eléctrico

    // Verificación de conexión I2C (WHO_AM_I) - Comprobación de cables
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
        uint8_t whoami = Wire.read();
        if (whoami == 0x68) {
            Serial.println("[MPU] Sensor detectado correctamente!");
        } else {
            Serial.printf("[MPU] ATENCIÓN: Dispositivo detectado pero WHO_AM_I es 0x%02X (Debería ser 0x68)\n", whoami);
        }
    } else {
        Serial.println("[ERROR CRÍTICO] MPU6050 NO ENCONTRADO. ¡Revisa que SDA esté en el pin 8, SCL en el pin 9, VCC y GND!");
    }

    mpu_write(REG_PWR_MGMT_1,  0x01);  // Despertar
    mpu_write(REG_SMPLRT_DIV,  0x00);  // Rate = 1kHz
    mpu_write(REG_ACCEL_CONFIG, 0x08); // ±4g: 8192 LSB/g
    mpu_write(REG_INT_PIN_CFG, 0x10);  // INT Active High
    mpu_write(REG_INT_ENABLE,  0x01);  // DATA_RDY_EN

    Serial.println("[MPU] Inicializado: 1kHz");
}

float leerCorrienteRMS() {
    /*--------------------------------------------------
      1) Obtener punto medio real del ADC
    --------------------------------------------------*/
    uint32_t sumaCentro = 0;
    for (int i = 0; i < 500; i++) {
        sumaCentro += analogRead(PIN_ADC);
        delayMicroseconds(50);
    }
    int puntoCentro = sumaCentro / 500;

    /*--------------------------------------------------
      2) Medir señal
    --------------------------------------------------*/
    uint64_t sumaCuadrados = 0;
    int minimo = 4095;
    int maximo = 0;

    for (int i = 0; i < MUESTRAS; i++) {
        int lectura = analogRead(PIN_ADC);
        if (lectura < minimo) minimo = lectura;
        if (lectura > maximo) maximo = lectura;

        long muestra = lectura - puntoCentro;
        sumaCuadrados += (uint64_t)muestra * muestra;
        delayMicroseconds(100);
    }

    /*--------------------------------------------------
      4) Calcular RMS del ADC
    --------------------------------------------------*/
    double rmsADC = (double)sumaCuadrados / (double)MUESTRAS;
    if (rmsADC < 0 || isnan(rmsADC) || isinf(rmsADC)) return NAN;
    
    double voltajeRMS = sqrt(rmsADC) * (VOLTAJE_REF / ADC_RESOLUCION);

    /*--------------------------------------------------
      5) Convertir a corriente primaria
    --------------------------------------------------*/
    double corriente = (voltajeRMS / RESISTENCIA) * RATIO * FACTOR_CAL;

    /*--------------------------------------------------
      6) Validaciones
    --------------------------------------------------*/
    if (isnan(corriente) || isinf(corriente)) return NAN;
    if (corriente < UMBRAL_RUIDO) corriente = 0.0;

    return (float)corriente;
}
