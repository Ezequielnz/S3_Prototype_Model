/*
 * datalyx_nodo.ino — Nodo de adquisición Datalyx v4  (ESP32-S3 Super Mini + MPU6050 + SCT013 + W5500)
 *
 * QUÉ HACE
 *   1) Adquiere aceleración 3 ejes (MPU6050, 1 kHz, disparado por su pin INT) y corriente
 *      (SCT013, 4 kHz, ADC por DMA). Las dos señales llevan timestamp del MISMO reloj del ESP32
 *      (esp_timer, µs) tomado en interrupción -> correlación vibración/corriente del orden de µs.
 *   2) Cada ventana (1024 muestras ≈ 1,02 s) calcula en el propio micro:
 *        - velocidad RMS por eje en la banda ISO (10 Hz–F_HI) por FFT  -> zona ISO 20816 A/B/C/D
 *        - velocidad RMS y pico por método temporal (verificación cruzada)
 *        - aceleración RMS, pico, factor de cresta, curtosis
 *        - frecuencia y amplitud dominante, componente a 2 x f_red
 *        - corriente RMS verdadera, fundamental, frecuencia de red, THD, pico, cresta
 *   3) Envía crudo (para entrenar la TCN) continuo o en ráfagas periódicas.
 *   Todo sale por Ethernet (W5500) en UDP binario. El USB queda para mensajes de diagnóstico.
 *
 * REPARTO DE NÚCLEOS
 *   Núcleo 1:  ISR INT MPU -> tarea "acq" (prioridad máxima, solo lee I2C)
 *              ISR ADC DMA -> cola de tramas con timestamp
 *              tarea "dsp" (cálculos, empaquetado)
 *   Núcleo 0:  lwIP + driver W5500, tareas "netTx" (envío) y "netRx" (comandos y sync de reloj)
 *
 * REQUISITOS
 *   - Arduino IDE con el core "esp32 by Espressif" >= 3.1 (probado con 3.3.12).
 *   - Placa: ESP32S3 Dev Module (o la que uses para la Super Mini), "USB CDC On Boot: Enabled".
 *   - Archivos dlx_proto.h y dlx_dsp.h en la misma carpeta que este .ino.
 *
 * CONEXIONES
 *   SCT013 (con polarización a ~1,65 V) -> GPIO 3
 *   MPU6050: SDA -> GPIO 8 | SCL -> GPIO 9 | INT -> GPIO 7
 *   W5500:   SCK -> GPIO 12 | MISO -> GPIO 13 | MOSI -> GPIO 11 | CS -> GPIO 10 | RST -> GPIO 5
 *            (INT del W5500 sin conectar: el driver trabaja por polling)
 *
 * PROTOCOLO: ver dlx_proto.h. Receptor de PC: datalyx_rx.py
 * COMANDOS (texto UDP al puerto PUERTO_ESP): HELLO | INFO | RAW ON | RAW OFF | RAW AUTO | RAW BURST <s>
 *                                            SYNC<8 bytes> (sincronización de reloj)
 */

#include <Arduino.h>
#include <Wire.h>
#include <ETH.h>
#include "lwip/sockets.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "dlx_proto.h"
#include "dlx_dsp.h"

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Se necesita el core esp32 de Espressif >= 3.1 (Herramientas > Placa > Gestor de placas)"
#endif

#define FW_VERSION "dlx-nodo-4.0"

// ============================== PINES ===============================
#define PIN_CT        3
#define PIN_SDA       8
#define PIN_SCL       9
#define PIN_INT       7
#define PIN_ETH_SCK   12
#define PIN_ETH_MISO  13
#define PIN_ETH_MOSI  11
#define PIN_ETH_CS    10
#define PIN_ETH_RST   5
#define ETH_SPI_MHZ   20     // 20 MHz es seguro con cables dupont; con PCB se puede subir a 33-40

// ============================== RED =================================
// Cable directo PC <-> ESP32: dejar USAR_DHCP 0 y poner en la placa de red de la PC
// IP 192.168.10.10, máscara 255.255.255.0 (sin puerta de enlace).
// Si conectás a un router/switch con DHCP: USAR_DHCP 1 (la IP aparece por el monitor serie).
#define USAR_DHCP     0
#define IP_ESP        192, 168, 10, 50
#define IP_GW         192, 168, 10, 1
#define IP_MASK       255, 255, 255, 0
#define IP_PC         192, 168, 10, 10   // destino inicial; cualquier comando de la PC lo actualiza
#define PUERTO_PC     5005               // donde escucha datalyx_rx.py
#define PUERTO_ESP    5006               // donde el ESP32 escucha comandos

// ============================ MPU6050 ===============================
#define MPU_ADDR      0x68   // 0x68 si AD0 a GND, 0x69 si AD0 a 3,3 V
#define ACCEL_RANGO   2      // 0=±2g 1=±4g 2=±8g 3=±16g  (menor rango = mejor resolución)
#define FS_ACC_NOM    1000.0f

// ============================ CORRIENTE =============================
#define FS_CUR_HZ     4000   // muestreo del SCT013 (80 muestras por ciclo de 50 Hz, armónicos hasta 2 kHz)
#define ADC_FRAME_N   128    // muestras por trama DMA (32 ms a 4 kHz)
#define CAL_FACTOR    0.73   // CAL nuevo = CAL actual × (valor pinza / valor equipo). Calibrar con carga alta.
#define UMBRAL_RUIDO_A 0.30  // por debajo, se considera motor apagado (bit ST_MOTOR_ON)
#define USAR_SCT013_000 1    // 1 = SCT013-000 (100A:50mA + burden externa) | 0 = SCT013-030 (30A/1V)
#if USAR_SCT013_000
  #define BURDEN_OHMS   68.0
  #define CT_RELACION   2000.0
  #define AMPS_POR_VOLT (CT_RELACION / BURDEN_OHMS)
#else
  #define AMPS_POR_VOLT 30.0
#endif

// ========================= VENTANA E ISO 20816 ======================
#define VENT_N        1024   // muestras de aceleración por ventana (potencia de 2 para la FFT)
#define CALENTAMIENTO 3      // ventanas iniciales marcadas como no válidas (filtros estabilizándose)
#define ISO_F_LO      10.0f  // Hz. ISO 20816: 10 Hz para máquinas > 600 rpm (2 Hz para 120-600 rpm)
#define ISO_F_HI      250.0f // Hz. La norma pide 1000 Hz; el MPU6050 llega a ~260 Hz -> se limita aquí
#define VEL_TD_FC     4.0f   // Hz. Corte de los pasa-altos del método temporal
// Límites de zona (mm/s RMS). Por defecto: Clase I de la tabla clásica ISO 10816-1
// (máquinas pequeñas hasta 15 kW), referencia también usada con ISO 20816-1.
// Otras opciones típicas (ISO 20816-3, motores > 15 kW):
//   Grupo 2 (15-300 kW) base rígida: 1.4 / 2.8 / 4.5   | base flexible: 2.3 / 4.5 / 7.1
//   Grupo 1 (>300 kW)   base rígida: 2.3 / 4.5 / 7.1   | base flexible: 3.5 / 7.1 / 11.0
#define ISO_AB        0.71f
#define ISO_BC        1.80f
#define ISO_CD        4.50f

// ============================= CRUDO ================================
#define RAW_MODO      2      // 0 = no enviar | 1 = continuo | 2 = ráfagas periódicas
#define RAW_RAFAGA_S  10     // duración de cada ráfaga (s)
#define RAW_PERIODO_S 60     // una ráfaga cada este tiempo (s)
#define RAW_ACC_N     100    // muestras de aceleración por paquete (100 ms)

#define SERIAL_DEBUG  1      // 1 = imprime un resumen por ventana en el monitor serie

// ====================== DERIVADOS Y COMPROBACIONES ==================
constexpr int CUR_MAX  = (int)(VENT_N * (FS_CUR_HZ / FS_ACC_NOM) * 1.05f) + ADC_FRAME_N;
constexpr int NFFT_CUR = dlx::potencia2((int)(VENT_N * (FS_CUR_HZ / FS_ACC_NOM)));
constexpr int CUR_RING = 2 * (int)(VENT_N * (FS_CUR_HZ / FS_ACC_NOM)) / ADC_FRAME_N + 8;
static_assert((VENT_N & (VENT_N - 1)) == 0, "VENT_N debe ser potencia de 2");
static_assert(NFFT_CUR <= 4096, "FFT de corriente > 4096: subir CONFIG_DSP_MAX_FFT_SIZE o bajar FS_CUR_HZ");
static_assert(sizeof(DlxHdr) + RAW_ACC_N * sizeof(DlxAccSample) <= DLX_MAX_PKT, "RAW_ACC_N muy grande");

// ============================== TIPOS ===============================
struct MuestraAcc {          // 16 bytes
  int64_t t;                 // µs, flanco de INT
  int16_t x, y, z, temp;     // cuentas crudas
};

struct TramaADC {            // la arma la ISR del ADC
  int64_t  tUltima;          // µs al completar la trama (= última muestra)
  uint16_t n;
  uint16_t raw[ADC_FRAME_N];
};

struct TramaCur {            // trama ya convertida a mV, guardada en el anillo
  int64_t  t0;               // µs de la primera muestra
  float    ts;               // µs entre muestras
  uint16_t n;
  bool     recorte;
  uint16_t mv[ADC_FRAME_N];
};

struct PaqueteRed {
  uint16_t len;
  uint8_t  d[DLX_MAX_PKT];
};

struct Ventana {
  int16_t  x[VENT_N], y[VENT_N], z[VENT_N];
  int      n;
  int64_t  t0, tUlt;
  bool     saturado, huecos;
  int32_t  tempSum;
  double   v2[3];
  float    vpk[3];
};

// ============================ GLOBALES ==============================
static QueueHandle_t qAcc, qAdc, qRed;
static TaskHandle_t  hAcq = nullptr;
static portMUX_TYPE  muxInt  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE  muxDest = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t tIntUs = 0;

static adc_continuous_handle_t adcH = nullptr;
static adc_channel_t adcCanal;
static uint16_t mvLut[4096];             // cuenta ADC -> mV (calibración de fábrica del chip)

// Contadores compartidos entre ISR y tareas: se incrementan con operaciones atómicas
static uint32_t cntAccPerdidas = 0, cntAdcPerdidas = 0, cntRedPerdidas = 0, cntI2cErr = 0;
#define SUMAR(c, k) __atomic_fetch_add(&(c), (uint32_t)(k), __ATOMIC_RELAXED)
static volatile bool     intFunciona = true;
static volatile bool     ethArriba = false;
static volatile bool     pedirInfo = true;

static int sockTx = -1, sockRx = -1;
static sockaddr_in destino;

static volatile uint8_t  rawModo = RAW_MODO;
static volatile int64_t  rawUnaVezHasta = 0;
static volatile uint16_t rawEpoca = 1;

static const float LSB_POR_G[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
static const float ESC_ACC = 9.80665f / LSB_POR_G[ACCEL_RANGO];          // m/s² por cuenta
static const float AMP_POR_MV = (float)(AMPS_POR_VOLT * CAL_FACTOR / 1000.0);

// ====================== ENVÍO (cola hacia netTx) ====================
static bool encolar(PaqueteRed &p, uint8_t tipo, uint16_t n, int64_t t0, uint16_t rafaga,
                    uint32_t &seq, uint16_t lenPayload) {
  DlxHdr *h = (DlxHdr *)p.d;
  h->magic = DLX_MAGIC; h->type = tipo; h->version = DLX_VERSION; h->n = n;
  h->seq = seq++; h->burst = rafaga; h->flags = 0; h->t0_us = t0;
  p.len = sizeof(DlxHdr) + lenPayload;
  if (xQueueSend(qRed, &p, 0) != pdTRUE) { SUMAR(cntRedPerdidas, 1); return false; }
  return true;
}

static void texto(const char *msg) {
  static uint32_t seqTxt = 0;
  PaqueteRed p;
  size_t L = strnlen(msg, 200);
  memcpy(p.d + sizeof(DlxHdr), msg, L);
  encolar(p, DLX_TEXT, (uint16_t)L, esp_timer_get_time(), 0, seqTxt, (uint16_t)L);
  Serial.printf("# %s\n", msg);
}

// ============================ MPU6050 ===============================
static bool escribirReg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}

static bool leerRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool iniciarMPU6050() {
  uint8_t id;
  if (!leerRegs(0x75, &id, 1)) return false;            // WHO_AM_I
  Serial.printf("# MPU6050 WHO_AM_I = 0x%02X\n", id);
  escribirReg(0x6B, 0x80); delay(100);                   // reset
  escribirReg(0x6B, 0x01); delay(100);                   // despierta, reloj = PLL del giróscopo X
  escribirReg(0x1A, 0x00);   // CONFIG: DLPF off -> acelerómetro 260 Hz de ancho de banda, retardo ~0 ms
  escribirReg(0x19, 7);      // SMPLRT_DIV: 8 kHz / (1+7) = 1 kHz (el acelerómetro entrega 1 kHz máx.)
  escribirReg(0x1C, ACCEL_RANGO << 3);
  escribirReg(0x37, 0x00);   // INT_PIN_CFG: activo alto, push-pull, pulso de 50 µs
  escribirReg(0x38, 0x01);   // INT_ENABLE: data ready
  return true;
}

// ISR del pin INT: solo toma la hora y despierta a la tarea de adquisición.
static void IRAM_ATTR isrMPU() {
  int64_t t = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&muxInt);
  tIntUs = t;
  portEXIT_CRITICAL_ISR(&muxInt);
  BaseType_t hp = pdFALSE;
  vTaskNotifyGiveFromISR(hAcq, &hp);
  if (hp) portYIELD_FROM_ISR();
}

// Tarea de máxima prioridad: por cada INT lee aceleración XYZ + temperatura (8 bytes, ~250 µs a 400 kHz).
static void tareaAcq(void *) {
  uint8_t b[8];
  bool polling = false;
  TickType_t espera = pdMS_TO_TICKS(300);       // al arrancar se da margen hasta que se conecte la ISR
  for (;;) {
    uint32_t c = ulTaskNotifyTake(pdTRUE, polling ? 1 : espera);
    espera = pdMS_TO_TICKS(50);
    int64_t t;
    if (c > 0) {
      if (polling) { polling = false; intFunciona = true; texto("INT del MPU6050 recuperada"); }
      if (c > 1) SUMAR(cntAccPerdidas, c - 1);     // llegaron 2+ INT antes de poder leer
      portENTER_CRITICAL(&muxInt); t = tIntUs; portEXIT_CRITICAL(&muxInt);
    } else {
      if (!polling) {
        polling = true; intFunciona = false;
        texto("AVISO: no llega INT (GPIO 7). Leyendo por polling: timestamps con jitter. Revisar cable.");
      }
      t = esp_timer_get_time();
    }
    if (!leerRegs(0x3B, b, 8)) { SUMAR(cntI2cErr, 1); SUMAR(cntAccPerdidas, 1); continue; }
    MuestraAcc s;
    s.t = t;
    s.x = (int16_t)((b[0] << 8) | b[1]);
    s.y = (int16_t)((b[2] << 8) | b[3]);
    s.z = (int16_t)((b[4] << 8) | b[5]);
    s.temp = (int16_t)((b[6] << 8) | b[7]);
    if (xQueueSend(qAcc, &s, 0) != pdTRUE) SUMAR(cntAccPerdidas, 1);
  }
}

// ====================== ADC CONTINUO (DMA) ==========================
// Se ejecuta en interrupción cada vez que el DMA completa una trama de ADC_FRAME_N muestras.
// La hora se toma acá: es la de la última muestra (+ unos µs de latencia).
static bool IRAM_ATTR adcTramaLista(adc_continuous_handle_t, const adc_continuous_evt_data_t *ed, void *) {
  TramaADC f;
  f.tUltima = esp_timer_get_time();
  uint32_t n = ed->size / SOC_ADC_DIGI_RESULT_BYTES;
  if (n > ADC_FRAME_N) n = ADC_FRAME_N;
  const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)ed->conv_frame_buffer;
  uint16_t k = 0;
  for (uint32_t i = 0; i < n; i++)
    if (p[i].type2.channel == adcCanal) f.raw[k++] = p[i].type2.data;
  f.n = k;
  BaseType_t hp = pdFALSE;
  if (xQueueSendFromISR(qAdc, &f, &hp) != pdTRUE) SUMAR(cntAdcPerdidas, 1);
  return hp == pdTRUE;
}

static bool iniciarADC() {
  adc_unit_t unidad;
  if (adc_continuous_io_to_channel(PIN_CT, &unidad, &adcCanal) != ESP_OK || unidad != ADC_UNIT_1) {
    Serial.println("# ERROR: PIN_CT no es un canal del ADC1");
    return false;
  }
  adc_continuous_handle_cfg_t hc = {};
  hc.max_store_buf_size = ADC_FRAME_N * SOC_ADC_DIGI_RESULT_BYTES * 4;
  hc.conv_frame_size    = ADC_FRAME_N * SOC_ADC_DIGI_RESULT_BYTES;
  hc.flags.flush_pool   = 1;    // los datos se copian en la ISR; el pool interno no se usa
  if (adc_continuous_new_handle(&hc, &adcH) != ESP_OK) return false;

  adc_digi_pattern_config_t pat = {};
  pat.atten = ADC_ATTEN_DB_12;           // rango ~0-3,1 V
  pat.channel = adcCanal;
  pat.unit = ADC_UNIT_1;
  pat.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  adc_continuous_config_t cc = {};
  cc.pattern_num = 1;
  cc.adc_pattern = &pat;
  cc.sample_freq_hz = FS_CUR_HZ;
  cc.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  cc.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  if (adc_continuous_config(adcH, &cc) != ESP_OK) return false;

  adc_continuous_evt_cbs_t cbs = {};
  cbs.on_conv_done = adcTramaLista;
  if (adc_continuous_register_event_callbacks(adcH, &cbs, nullptr) != ESP_OK) return false;

  // Tabla de calibración (curva de fábrica guardada en eFuse del chip)
  adc_cali_handle_t cal = nullptr;
  adc_cali_curve_fitting_config_t cf = {};
  cf.unit_id = ADC_UNIT_1;
  cf.chan = adcCanal;
  cf.atten = ADC_ATTEN_DB_12;
  cf.bitwidth = ADC_BITWIDTH_12;
  bool calOk = adc_cali_create_scheme_curve_fitting(&cf, &cal) == ESP_OK;
  for (int r = 0; r < 4096; r++) {
    int mv = r * 3100 / 4095;
    if (calOk) adc_cali_raw_to_voltage(cal, r, &mv);
    mvLut[r] = (uint16_t)mv;
  }
  if (calOk) adc_cali_delete_scheme_curve_fitting(cal);
  Serial.printf("# ADC: canal %d, %d Hz, calibracion %s\n", (int)adcCanal, FS_CUR_HZ, calOk ? "eFuse OK" : "NO (lineal)");
  return adc_continuous_start(adcH) == ESP_OK;
}

// ========================== CRUDO: CUÁNDO ===========================
// Se decide por el timestamp de cada muestra, así acc y corriente cortan en el mismo instante.
static bool rawActivo(int64_t t, uint16_t &rafaga) {
  if (t < rawUnaVezHasta) { rafaga = rawEpoca; return true; }
  switch (rawModo) {
    case 1: rafaga = rawEpoca; return true;
    case 2: {
      int64_t per = (int64_t)RAW_PERIODO_S * 1000000LL;
      rafaga = (uint16_t)(t / per);
      return (t % per) < (int64_t)RAW_RAFAGA_S * 1000000LL;
    }
    default: return false;
  }
}

// ============================ TAREA DSP =============================
static Ventana   ven[2];
static int       vLlenando = 0, vPendiente = -1;
static TramaCur  anillo[CUR_RING];
static int       anCabeza = 0, anCuenta = 0;
static float     bufA[VENT_N];
static float     bufI[CUR_MAX];
static float     work[2 * NFFT_CUR] __attribute__((aligned(16)));
static dlx::CadenaVelocidad vel[3];

// Estimación de largo plazo de las frecuencias de muestreo reales (el MPU6050 usa su propio oscilador)
struct EstimFs {
  int64_t tRef = -1; uint64_t n = 0; float fs;
  void reset() { tRef = -1; n = 0; }
  void muestra(int64_t t, uint32_t k = 1) {
    if (tRef < 0) { tRef = t; n = 0; return; }
    n += k;
    if (t - tRef > 2000000) fs = (float)(n * 1e6 / (double)(t - tRef));
  }
};
static EstimFs fsAcc, fsCur;

static PaqueteRed pktAcc, pktCur, pktVar;
static uint32_t   seqAcc = 0, seqCur = 0, seqFeat = 0, seqInfo = 0;
static int        nPktAcc = 0;
static int64_t    t0PktAcc = 0;
static uint16_t   rafPktAcc = 0;
static uint32_t   iVentana = 0;
static float      fLineaUlt = 0;

static void flushRawAcc() {
  if (nPktAcc == 0) return;
  encolar(pktAcc, DLX_RAW_ACC, nPktAcc, t0PktAcc, rafPktAcc, seqAcc, nPktAcc * sizeof(DlxAccSample));
  nPktAcc = 0;
}

static void enviarInfo() {
  DlxInfo in = {};
  strncpy(in.fw, FW_VERSION, sizeof(in.fw) - 1);
  in.fs_acc_nom = FS_ACC_NOM; in.fs_cur_nom = FS_CUR_HZ;
  in.acc_ms2_per_lsb = ESC_ACC; in.amp_per_mv = AMP_POR_MV;
  in.win_n = VENT_N; in.adc_frame_n = ADC_FRAME_N;
  in.iso_flo = ISO_F_LO; in.iso_fhi = ISO_F_HI;
  in.iso_ab = ISO_AB; in.iso_bc = ISO_BC; in.iso_cd = ISO_CD;
  in.raw_mode = rawModo; in.accel_range = ACCEL_RANGO;
  in.raw_burst_s = RAW_RAFAGA_S; in.raw_period_s = RAW_PERIODO_S;
  memcpy(pktVar.d + sizeof(DlxHdr), &in, sizeof(in));
  encolar(pktVar, DLX_INFO, 1, esp_timer_get_time(), 0, seqInfo, sizeof(in));
}

static void procesarTramaADC(const TramaADC &f) {
  static int64_t tFinAnt = -1;
  if (f.n == 0) return;
  float ts = 1e6f / fsCur.fs;
  int64_t t0 = f.tUltima - (int64_t)lroundf((f.n - 1) * ts);
  if (tFinAnt > 0 && t0 - tFinAnt > 4 * ts) fsCur.reset();   // se perdió alguna trama
  tFinAnt = f.tUltima;
  fsCur.muestra(f.tUltima, f.n);

  TramaCur &c = anillo[anCabeza];
  c.t0 = t0; c.ts = ts; c.n = f.n; c.recorte = false;
  for (int i = 0; i < f.n; i++) {
    uint16_t r = f.raw[i] & 0x0FFF;
    if (r <= 5 || r >= 4090) c.recorte = true;
    c.mv[i] = mvLut[r];
  }
  anCabeza = (anCabeza + 1) % CUR_RING;
  if (anCuenta < CUR_RING) anCuenta++;

  // Crudo: se mandan las muestras de la trama que caen dentro de la ráfaga
  uint16_t raf = 0;
  int iIni = -1, iFin = -1;
  for (int i = 0; i < c.n; i++) {
    uint16_t r2;
    if (rawActivo(c.t0 + (int64_t)lroundf(i * ts), r2)) { if (iIni < 0) { iIni = i; raf = r2; } iFin = i; }
  }
  if (iIni >= 0) {
    int n = iFin - iIni + 1;
    DlxCurHdr ch; ch.fs_hz = fsCur.fs;
    memcpy(pktCur.d + sizeof(DlxHdr), &ch, sizeof(ch));
    memcpy(pktCur.d + sizeof(DlxHdr) + sizeof(ch), &c.mv[iIni], n * 2);
    encolar(pktCur, DLX_RAW_CUR, n, c.t0 + (int64_t)lroundf(iIni * ts), raf, seqCur, sizeof(ch) + n * 2);
  }
}

// ¿Ya llegó corriente que cubra hasta tFin?
static bool corrienteCubre(int64_t tFin) {
  if (anCuenta == 0) return false;
  const TramaCur &u = anillo[(anCabeza - 1 + CUR_RING) % CUR_RING];
  return u.t0 + (int64_t)((u.n - 1) * u.ts) >= tFin;
}

// Junta las muestras de corriente con timestamp en [tIni, tFin). Devuelve cuántas.
static int juntarCorriente(int64_t tIni, int64_t tFin, bool &recorte, bool &huecos) {
  int m = 0;
  recorte = false; huecos = false;
  int64_t tEsperado = -1;
  for (int j = 0; j < anCuenta; j++) {
    const TramaCur &c = anillo[(anCabeza - anCuenta + j + CUR_RING) % CUR_RING];
    int64_t tFinTrama = c.t0 + (int64_t)((c.n - 1) * c.ts);
    if (tFinTrama < tIni || c.t0 >= tFin) continue;
    if (tEsperado > 0 && c.t0 - tEsperado > 4 * c.ts) huecos = true;
    tEsperado = tFinTrama + (int64_t)c.ts;
    for (int i = 0; i < c.n && m < CUR_MAX; i++) {
      int64_t t = c.t0 + (int64_t)(i * c.ts);
      if (t >= tIni && t < tFin) { bufI[m++] = c.mv[i]; recorte |= c.recorte; }
    }
  }
  return m;
}

static void calcularVentana(Ventana &w) {
  int64_t tc0 = esp_timer_get_time();
  DlxFeat F = {};
  const int M = w.n;
  const float fs = fsAcc.fs;
  const float tsAcc = 1e6f / fs;
  const int64_t tFin = w.tUlt + (int64_t)tsAcc;
  uint16_t st = 0;

  // ---------- Corriente (primero: da la frecuencia de red para el 2xf) ----------
  bool recorte, huecos;
  int nI = juntarCorriente(w.t0, tFin, recorte, huecos);
  dlx::IndCorriente ic;
  dlx::indicadoresCorriente(bufI, nI, NFFT_CUR, fsCur.fs, AMP_POR_MV, work, ic);
  if (nI < 64) st |= ST_SIN_CORRIENTE;
  if (recorte) st |= ST_CUR_RECORTE;
  if (huecos || nI < (int)(0.95f * M * fsCur.fs / fs)) st |= ST_CUR_HUECOS;
  if (ic.i1_rms > UMBRAL_RUIDO_A) st |= ST_MOTOR_ON;
  if (ic.f_line > 0) fLineaUlt = ic.f_line;

  // ---------- Vibración por eje ----------
  const int16_t *ejes[3] = {w.x, w.y, w.z};
  float fHi = ISO_F_HI < 0.45f * fs ? ISO_F_HI : 0.45f * fs;
  float vMax = 0; uint8_t ejeMax = 0;
  for (int e = 0; e < 3; e++) {
    for (int i = 0; i < M; i++) bufA[i] = ejes[e][i] * ESC_ACC;
    dlx::indicadoresEje(bufA, M, VENT_N, fs, ISO_F_LO, fHi, fLineaUlt, work, F.ax[e]);
    F.ax[e].v_rms_td = (float)sqrt(w.v2[e] / M);
    F.ax[e].v_pk_td  = w.vpk[e];
    if (F.ax[e].v_rms > vMax) { vMax = F.ax[e].v_rms; ejeMax = e; }
  }

  if (iVentana >= CALENTAMIENTO) st |= ST_VALIDA;
  if (w.saturado) st |= ST_ACC_SATURADO;
  if (w.huecos)   st |= ST_ACC_PERDIDAS;
  if (!intFunciona) st |= ST_SIN_INT;
  uint16_t rTmp;
  if (rawActivo(w.tUlt, rTmp)) st |= ST_RAW_ACTIVO;

  F.window = iVentana; F.t_end_us = tFin;
  F.fs_acc = fs; F.fs_cur = fsCur.fs;
  F.n_acc = M; F.n_cur = nI; F.status = st;
  F.v_iso = vMax; F.iso_axis = ejeMax;
  F.iso_zone = dlx::zonaISO(vMax, ISO_AB, ISO_BC, ISO_CD);
  F.temp_c = (float)w.tempSum / M / 340.0f + 36.53f;
  F.i_rms = ic.i_rms; F.i_pk = ic.i_pk; F.i_crest = ic.i_crest; F.i1_rms = ic.i1_rms;
  F.f_line = ic.f_line; F.thd = ic.thd; F.bias_mv = ic.bias_mv;
  F.acc_missed = cntAccPerdidas; F.adc_dropped = cntAdcPerdidas; F.net_dropped = cntRedPerdidas;
  F.dsp_ms = (esp_timer_get_time() - tc0) / 1000.0f;

  memcpy(pktVar.d + sizeof(DlxHdr), &F, sizeof(F));
  encolar(pktVar, DLX_FEAT, 1, w.t0, 0, seqFeat, sizeof(F));

  // Ajuste fino del integrador con la fs medida
  for (int e = 0; e < 3; e++) vel[e].ajustarFs(fs);

#if SERIAL_DEBUG
  if (Serial && Serial.availableForWrite() > 200) {
    static const char Z[4] = {'A', 'B', 'C', 'D'};
    Serial.printf("#W%lu %s v_iso=%.2f mm/s [%c, eje %c] | vX=%.2f vY=%.2f vZ=%.2f | I=%.2f A (I1=%.2f) f=%.2f Hz THD=%.1f%% | "
                  "fsA=%.2f fsI=%.1f nI=%d | st=0x%03X perd=%lu/%lu/%lu | %.2f ms\n",
                  (unsigned long)iVentana, (st & ST_VALIDA) ? "" : "(calentando)",
                  vMax, Z[F.iso_zone], "XYZ"[ejeMax], F.ax[0].v_rms, F.ax[1].v_rms, F.ax[2].v_rms,
                  ic.i_rms, ic.i1_rms, ic.f_line, ic.thd * 100, fs, fsCur.fs, nI, st,
                  (unsigned long)cntAccPerdidas, (unsigned long)cntAdcPerdidas, (unsigned long)cntRedPerdidas, F.dsp_ms);
  }
#endif
  iVentana++;
}

static void procesarAcc(const MuestraAcc &s) {
  static int64_t tAnt = -1;
  float ts = 1e6f / fsAcc.fs;
  bool hueco = tAnt > 0 && (s.t - tAnt) > 1.5f * ts;
  if (hueco) fsAcc.reset();
  tAnt = s.t;
  fsAcc.muestra(s.t);

  Ventana &w = ven[vLlenando];
  if (w.n == 0) {
    w.t0 = s.t; w.saturado = false; w.huecos = false; w.tempSum = 0;
    for (int e = 0; e < 3; e++) { w.v2[e] = 0; w.vpk[e] = 0; }
  }
  w.huecos |= hueco;
  int i = w.n;
  w.x[i] = s.x; w.y[i] = s.y; w.z[i] = s.z;
  w.tempSum += s.temp;
  if (s.x == 32767 || s.x == -32768 || s.y == 32767 || s.y == -32768 || s.z == 32767 || s.z == -32768)
    w.saturado = true;

  // Velocidad, método temporal (estado continuo entre ventanas)
  static bool primera = true;
  float a[3] = {s.x * ESC_ACC, s.y * ESC_ACC, s.z * ESC_ACC};
  for (int e = 0; e < 3; e++) {
    if (primera) vel[e].precargar(a[e]);
    float v = vel[e].paso(a[e]) * 1000.0f;   // mm/s
    w.v2[e] += (double)v * v;
    if (fabsf(v) > w.vpk[e]) w.vpk[e] = fabsf(v);
  }
  primera = false;
  w.tUlt = s.t;
  w.n++;

  // Crudo
  uint16_t raf;
  if (rawActivo(s.t, raf)) {
    if (nPktAcc > 0 && (raf != rafPktAcc || s.t - t0PktAcc > 4000000000LL)) flushRawAcc();
    if (nPktAcc == 0) { t0PktAcc = s.t; rafPktAcc = raf; }
    DlxAccSample *ps = (DlxAccSample *)(pktAcc.d + sizeof(DlxHdr)) + nPktAcc;
    ps->dt_us = (uint32_t)(s.t - t0PktAcc);
    ps->ax = s.x; ps->ay = s.y; ps->az = s.z;
    if (++nPktAcc >= RAW_ACC_N) flushRawAcc();
  } else {
    flushRawAcc();
  }

  // ¿Ventana completa?
  if (w.n >= VENT_N) {
    if (vPendiente >= 0) calcularVentana(ven[vPendiente]);   // la corriente nunca llegó: calcular igual
    vPendiente = vLlenando;
    vLlenando ^= 1;
    ven[vLlenando].n = 0;
  }
}

static void tareaDSP(void *) {
  fsAcc.fs = FS_ACC_NOM;
  fsCur.fs = FS_CUR_HZ;
  for (int e = 0; e < 3; e++) vel[e].iniciar(VEL_TD_FC, FS_ACC_NOM);
  int64_t tInfo = 0;
  TramaADC f;
  MuestraAcc s;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(4));
    while (xQueueReceive(qAdc, &f, 0) == pdTRUE) procesarTramaADC(f);
    while (xQueueReceive(qAcc, &s, 0) == pdTRUE) procesarAcc(s);
    if (vPendiente >= 0) {
      Ventana &w = ven[vPendiente];
      if (corrienteCubre(w.tUlt + (int64_t)(1e6f / fsAcc.fs))) {
        calcularVentana(w);
        vPendiente = -1;
      }
    }
    int64_t ahora = esp_timer_get_time();
    if (pedirInfo || ahora - tInfo > 5000000) { pedirInfo = false; tInfo = ahora; enviarInfo(); }
  }
}

// ============================== RED =================================
static void tareaRedTx(void *) {
  static PaqueteRed p;
  for (;;) {
    if (xQueueReceive(qRed, &p, portMAX_DELAY) != pdTRUE) continue;
    if (sockTx < 0) { SUMAR(cntRedPerdidas, 1); continue; }
    sockaddr_in d;
    portENTER_CRITICAL(&muxDest); d = destino; portEXIT_CRITICAL(&muxDest);
    if (sendto(sockTx, p.d, p.len, 0, (sockaddr *)&d, sizeof(d)) < 0) SUMAR(cntRedPerdidas, 1);
  }
}

static void tareaRedRx(void *) {
  uint8_t buf[128];
  for (;;) {
    sockaddr_in src; socklen_t sl = sizeof(src);
    int r = recvfrom(sockRx, buf, sizeof(buf) - 1, 0, (sockaddr *)&src, &sl);
    int64_t tRx = esp_timer_get_time();
    if (r <= 0) { vTaskDelay(10); continue; }

    if (r >= 12 && memcmp(buf, "SYNC", 4) == 0) {   // respuesta inmediata, sin pasar por la cola
      uint8_t out[sizeof(DlxHdr) + sizeof(DlxSync)];
      static uint32_t seqSync = 0;
      DlxHdr *h = (DlxHdr *)out;
      DlxSync *sy = (DlxSync *)(out + sizeof(DlxHdr));
      h->magic = DLX_MAGIC; h->type = DLX_SYNC; h->version = DLX_VERSION; h->n = 1;
      h->seq = seqSync++; h->burst = 0; h->flags = 0; h->t0_us = tRx;
      memcpy(sy->pc_token, buf + 4, 8);
      sy->t_rx_us = tRx;
      sy->t_tx_us = esp_timer_get_time();
      sendto(sockRx, out, sizeof(out), 0, (sockaddr *)&src, sl);
      continue;
    }

    // Cualquier comando fija el destino de los datos (IP de la PC; puerto PUERTO_PC)
    src.sin_port = htons(PUERTO_PC);
    portENTER_CRITICAL(&muxDest); destino = src; portEXIT_CRITICAL(&muxDest);

    buf[r] = 0;
    char *cmd = (char *)buf;
    while (r > 0 && (cmd[r - 1] == '\n' || cmd[r - 1] == '\r' || cmd[r - 1] == ' ')) cmd[--r] = 0;
    char msg[96];
    if (!strcmp(cmd, "HELLO") || !strcmp(cmd, "INFO")) {
      pedirInfo = true;
      snprintf(msg, sizeof(msg), "Hola %s. Enviando datos a ese destino.", inet_ntoa(src.sin_addr));
      texto(msg);
    } else if (!strcmp(cmd, "RAW ON"))   { rawModo = 1; rawEpoca = rawEpoca + 1; rawUnaVezHasta = 0; texto("Crudo: continuo"); pedirInfo = true; }
    else if (!strcmp(cmd, "RAW OFF"))  { rawModo = 0; rawUnaVezHasta = 0; texto("Crudo: apagado"); pedirInfo = true; }
    else if (!strcmp(cmd, "RAW AUTO")) { rawModo = 2; rawUnaVezHasta = 0; texto("Crudo: rafagas periodicas"); pedirInfo = true; }
    else if (!strncmp(cmd, "RAW BURST", 9)) {
      int seg = atoi(cmd + 9); if (seg <= 0) seg = RAW_RAFAGA_S; if (seg > 3600) seg = 3600;
      rawEpoca = rawEpoca + 1;
      rawUnaVezHasta = esp_timer_get_time() + (int64_t)seg * 1000000LL;
      snprintf(msg, sizeof(msg), "Crudo: rafaga unica de %d s (id %u)", seg, (unsigned)rawEpoca);
      texto(msg);
    } else {
      snprintf(msg, sizeof(msg), "Comando desconocido: %.40s", cmd);
      texto(msg);
    }
  }
}

static void onRed(arduino_event_id_t ev, arduino_event_info_t) {
  switch (ev) {
    case ARDUINO_EVENT_ETH_CONNECTED:    Serial.println("# ETH: enlace arriba"); break;
    case ARDUINO_EVENT_ETH_GOT_IP:       ethArriba = true; Serial.printf("# ETH: IP %s\n", ETH.localIP().toString().c_str()); break;
    case ARDUINO_EVENT_ETH_DISCONNECTED: ethArriba = false; Serial.println("# ETH: cable desconectado"); break;
    case ARDUINO_EVENT_ETH_STOP:         ethArriba = false; Serial.println("# ETH: detenido"); break;
    default: break;
  }
}

static bool iniciarRed() {
  Network.onEvent(onRed);
  if (!ETH.begin(ETH_PHY_W5500, 1, PIN_ETH_CS, -1, PIN_ETH_RST, SPI2_HOST,
                 PIN_ETH_SCK, PIN_ETH_MISO, PIN_ETH_MOSI, ETH_SPI_MHZ)) {
    Serial.println("# ERROR: no responde el W5500. Revisar SPI (10-13), RST (5), 3,3 V y GND.");
    return false;
  }
#if !USAR_DHCP
  ETH.config(IPAddress(IP_ESP), IPAddress(IP_GW), IPAddress(IP_MASK));
#endif
  memset(&destino, 0, sizeof(destino));
  destino.sin_family = AF_INET;
  destino.sin_port = htons(PUERTO_PC);
  destino.sin_addr.s_addr = (uint32_t)IPAddress(IP_PC);

  sockTx = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  sockRx = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  sockaddr_in loc = {};
  loc.sin_family = AF_INET;
  loc.sin_port = htons(PUERTO_ESP);
  loc.sin_addr.s_addr = htonl(INADDR_ANY);
  if (sockTx < 0 || sockRx < 0 || bind(sockRx, (sockaddr *)&loc, sizeof(loc)) < 0) {
    Serial.println("# ERROR: no se pudieron crear los sockets UDP");
    return false;
  }
  return true;
}

// ============================== SETUP ===============================
void setup() {
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 2000) delay(10);
  Serial.println("\n# " FW_VERSION " iniciando...");

  qAcc = xQueueCreate(256, sizeof(MuestraAcc));     // 256 ms de margen
  qAdc = xQueueCreate(16, sizeof(TramaADC));        // ~0,5 s de margen
  qRed = xQueueCreate(32, sizeof(PaqueteRed));
  if (!qAcc || !qAdc || !qRed || !dlx::fftInit(NFFT_CUR)) {
    Serial.println("# ERROR: memoria insuficiente");
    for (;;) delay(1000);
  }

  // Red primero (núcleo 0), así los avisos posteriores ya salen por Ethernet
  if (iniciarRed()) {
    xTaskCreatePinnedToCore(tareaRedTx, "netTx", 4096, nullptr, 12, nullptr, 0);
    xTaskCreatePinnedToCore(tareaRedRx, "netRx", 4096, nullptr, 13, nullptr, 0);
  }

  // Sensores (núcleo 1)
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  while (!iniciarMPU6050()) {
    Serial.println("# ERROR: no se detecta el MPU6050. Revisar SDA(8), SCL(9), 3,3 V y GND.");
    delay(1000);
  }
  xTaskCreatePinnedToCore(tareaDSP, "dsp", 8192, nullptr, 8, nullptr, 1);
  xTaskCreatePinnedToCore(tareaAcq, "acq", 6144, nullptr, configMAX_PRIORITIES - 2, &hAcq, 1);
  pinMode(PIN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), isrMPU, RISING);
  if (!iniciarADC()) texto("ERROR: no arranco el ADC continuo (corriente)");

  texto("Nodo listo: " FW_VERSION);
}

void loop() {
  vTaskDelay(portMAX_DELAY);   // todo corre en tareas
}
