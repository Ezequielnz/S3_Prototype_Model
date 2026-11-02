/*
 * Medidor de corriente (SCT013) + aceleración eje Z (MPU6050)
 * Placa: ESP32-S3 Super Mini
 *
 * Salida por Serial Monitor (CSV, 115200 baud):
 *   tiempo_s, az_ms2, corriente_A
 *
 * Conexiones:
 *   SCT013 (con circuito de polarización) -> GPIO 3
 *   MPU6050: SDA -> GPIO 8 | SCL -> GPIO 9 | INT -> GPIO 7
 *
 * Arduino IDE: Placa "ESP32S3 Dev Module" y
 *              "USB CDC On Boot: Enabled" (si no, el Serial no muestra nada).
 */

#include <calibration.h>
#include <Wire.h>
#include <math.h>

// ======================= PINES =======================
#define PIN_CT   3
#define PIN_SDA  8
#define PIN_SCL  9
#define PIN_INT  7

// ================= CONFIGURACIÓN MPU6050 =============
#define MPU_ADDR        0x68   // 0x68 si AD0 a GND, 0x69 si AD0 a 3.3V
#define ACCEL_RANGO     1      // 0=±2g, 1=±4g, 2=±8g, 3=±16g
#define RESTAR_GRAVEDAD false  // true: resta 1g al eje Z (solo si Z apunta vertical)
#define IMPRIMIR_CADA_N 1      // imprime 1 de cada N muestras (200 muestras/seg)

// ================= CONFIGURACIÓN SCT013 ==============
// Elegí UNA de las dos opciones:

// Opción A: SCT013-000 (100A:50mA, SIN resistencia interna).
// Necesita resistencia "burden" externa. Relación 100A/0.05A = 2000.
#define BURDEN_OHMS     33.0
#define CT_RELACION     2000.0
#define AMPS_POR_VOLT   (CT_RELACION / BURDEN_OHMS)


// ================= VARIABLES GLOBALES ================
const float G = 9.80665f;
const float LSB_POR_G[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
float escalaAccel;

volatile float    corrienteA   = 0.0f;   // la escribe la tarea de corriente
volatile bool     nuevaMuestra = false;  // la activa la interrupción del MPU6050
volatile uint32_t tMuestraUs   = 0;

// ==================== INTERRUPCIÓN ===================
void IRAM_ATTR isrMPU() {
  tMuestraUs   = micros();
  nuevaMuestra = true;
}

// ==================== FUNCIONES I2C ==================
void escribirReg(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission();
}

bool leerAccelZ(int16_t &raw) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3F);                       // ACCEL_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2) != 2) return false;
  uint8_t alto = Wire.read();
  uint8_t bajo = Wire.read();
  raw = (int16_t)((alto << 8) | bajo);
  return true;
}

bool iniciarMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);                       // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) != 1) return false;
  uint8_t id = Wire.read();
  Serial.printf("# MPU6050 WHO_AM_I = 0x%02X\n", id);

  escribirReg(0x6B, 0x01);                // PWR_MGMT_1: despierta, reloj = PLL gyro X
  delay(100);
  escribirReg(0x1A, 0x01);                // CONFIG: filtro pasa-bajos (~184 Hz), muestreo base 1 kHz
  escribirReg(0x19, 4);                   // SMPLRT_DIV: 1000/(1+4) = 200 Hz
  escribirReg(0x1C, ACCEL_RANGO << 3);    // ACCEL_CONFIG: rango
  escribirReg(0x37, 0x00);                // INT_PIN_CFG: activo alto, pulso corto
  escribirReg(0x38, 0x01);                // INT_ENABLE: interrupción "data ready"
  return true;
}

// ============ TAREA DE CORRIENTE (núcleo 0) ==========
void tareaCorriente(void *param) {
  for (;;) {
    double suma = 0, sumaCuad = 0;
    uint32_t n = 0;
    uint32_t inicio = millis();

    while (millis() - inicio < VENTANA_MS) {
      double mv = analogReadMilliVolts(PIN_CT);
      suma     += mv;
      sumaCuad += mv * mv;
      n++;
    }

    double media    = suma / n;                       // offset de polarización (~1650 mV)
    double varianza = sumaCuad / n - media * media;   // RMS de la componente AC
    if (varianza < 0) varianza = 0;
    double vrms = sqrt(varianza) / 1000.0;            // en volts

    float i = (float)(vrms * AMPS_POR_VOLT * CAL_FACTOR);
    if (i < UMBRAL_RUIDO_A) i = 0.0f;
    corrienteA = i;

    vTaskDelay(1);                                    // deja respirar al watchdog
  }
}

// ======================== SETUP ======================
void setup() {
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_CT, ADC_11db);          // rango ~0 a 3.1 V

  // I2C + MPU6050
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  escalaAccel = G / LSB_POR_G[ACCEL_RANGO];

  if (!iniciarMPU6050()) {
    Serial.println("# ERROR: no se detecta el MPU6050. Revisar cableado/direccion.");
    while (true) delay(1000);
  }

  pinMode(PIN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), isrMPU, RISING);

  // Tarea de corriente en el otro núcleo
  xTaskCreatePinnedToCore(tareaCorriente, "corriente", 4096, NULL, 1, NULL, 0);

  Serial.println("tiempo_s,az_ms2,corriente_A");
}

// ======================== LOOP =======================
void loop() {
  static bool     primera = true;
  static uint32_t ultimoUs = 0;
  static uint64_t tAcumUs  = 0;   // tiempo acumulado (sin desbordar a los 71 min)
  static uint32_t contador = 0;

  if (!nuevaMuestra) return;
  nuevaMuestra = false;
  uint32_t ahoraUs = tMuestraUs;

  if (primera) { ultimoUs = ahoraUs; primera = false; }
  tAcumUs += (uint32_t)(ahoraUs - ultimoUs);
  ultimoUs = ahoraUs;

  int16_t raw;
  if (!leerAccelZ(raw)) return;

  if (++contador % IMPRIMIR_CADA_N != 0) return;

  float az = raw * escalaAccel;                       // m/s²
  if (RESTAR_GRAVEDAD) az -= G;
  float i = corrienteA;

  Serial.printf("%.3f,%.3f,%.2f\n", tAcumUs / 1e6, az, i);
}
