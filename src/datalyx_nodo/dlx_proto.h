/*
 * dlx_proto.h — Protocolo binario UDP del nodo Datalyx (v1)
 *
 * Todos los paquetes: [DlxHdr 24 bytes][payload]. Little-endian, sin padding.
 * Timestamps: microsegundos del reloj del ESP32 (esp_timer_get_time(), desde el arranque).
 * El MISMO reloj marca la vibración y la corriente -> la correlación entre ambas
 * dentro del nodo es del orden de microsegundos.
 *
 * Este archivo lo usan el firmware y el simulador de prueba. El receptor Python
 * (datalyx_rx.py) replica estas estructuras con el módulo struct.
 */
#pragma once
#include <stdint.h>

#define DLX_MAGIC    0x31584C44u   // "DLX1" en memoria (little-endian)
#define DLX_VERSION  1
#define DLX_MAX_PKT  1100          // bytes máximos por paquete (< MTU 1472 de UDP)

enum : uint8_t {
  DLX_RAW_ACC = 1,   // muestras crudas del acelerómetro (3 ejes)
  DLX_RAW_CUR = 2,   // muestras crudas de corriente (mV del ADC)
  DLX_FEAT    = 3,   // indicadores calculados por ventana (ISO 20816, etc.)
  DLX_INFO    = 4,   // configuración y factores de escala
  DLX_SYNC    = 5,   // respuesta a pedido de sincronización de reloj
  DLX_TEXT    = 6,   // mensaje de texto (avisos)
};

// Bits de DlxFeat.status
enum : uint16_t {
  ST_VALIDA        = 1u << 0,  // filtros estabilizados (pasó el calentamiento)
  ST_ACC_SATURADO  = 1u << 1,  // alguna muestra tocó el fondo de escala -> subir ACCEL_RANGO
  ST_ACC_PERDIDAS  = 1u << 2,  // hubo huecos en la serie del acelerómetro en esta ventana
  ST_SIN_INT       = 1u << 3,  // no llega la interrupción INT: lectura por polling (timestamps peores)
  ST_CUR_RECORTE   = 1u << 4,  // la señal del SCT013 tocó 0 V o 3.1 V (corriente fuera de rango)
  ST_CUR_HUECOS    = 1u << 5,  // faltan muestras de corriente en la ventana
  ST_MOTOR_ON      = 1u << 6,  // corriente fundamental > UMBRAL_RUIDO_A
  ST_RAW_ACTIVO    = 1u << 7,  // se está enviando crudo
  ST_SIN_CORRIENTE = 1u << 8,  // ventana calculada sin datos de corriente
};

#pragma pack(push, 1)

struct DlxHdr {
  uint32_t magic;     // DLX_MAGIC
  uint8_t  type;      // DLX_*
  uint8_t  version;   // DLX_VERSION
  uint16_t n;         // cantidad de muestras/items en el payload
  uint32_t seq;       // contador por tipo de paquete (para detectar pérdidas)
  uint16_t burst;     // id de ráfaga de crudo (agrupa acc + corriente)
  uint16_t flags;     // reservado
  int64_t  t0_us;     // timestamp de referencia del paquete
};                     // 24 bytes

// DLX_RAW_ACC: n x DlxAccSample. t_muestra = t0_us + dt_us
struct DlxAccSample {
  uint32_t dt_us;
  int16_t  ax, ay, az;  // cuentas crudas del MPU6050 (multiplicar por acc_ms2_per_lsb)
};                       // 10 bytes

// DLX_RAW_CUR: payload = DlxCurHdr + n x uint16 (mV). t_i = t0_us + i * 1e6 / fs_hz
struct DlxCurHdr {
  float fs_hz;
};

struct DlxAxisFeat {
  float a_rms;      // m/s²  RMS de la aceleración vibratoria (sin gravedad), banda completa
  float a_pk;       // m/s²  pico de |a|
  float crest;      // a_pk / a_rms
  float kurt;       // curtosis (3 = gaussiana; >4-5 sugiere impactos)
  float v_rms_td;   // mm/s  RMS de velocidad, método temporal (filtros + integración)
  float v_pk_td;    // mm/s  pico de velocidad, método temporal
  float v_rms;      // mm/s  RMS de velocidad en la banda ISO, método espectral (FFT)  <- ISO 20816
  float f_dom;      // Hz    frecuencia dominante del espectro de velocidad
  float v_dom;      // mm/s  RMS de esa componente
  float v_2fl;      // mm/s  RMS de la componente a 2 x frecuencia de red (firma electromagnética)
};                   // 40 bytes

struct DlxFeat {
  uint32_t window;      // índice de ventana
  int64_t  t_end_us;    // fin de ventana (t0_us del header = inicio)
  float    fs_acc;      // Hz medida del acelerómetro
  float    fs_cur;      // Hz medida del ADC de corriente
  uint16_t n_acc;       // muestras de aceleración en la ventana
  uint16_t n_cur;       // muestras de corriente en la ventana
  uint16_t status;      // bits ST_*
  uint8_t  iso_zone;    // 0=A 1=B 2=C 3=D
  uint8_t  iso_axis;    // eje con mayor v_rms (0=X 1=Y 2=Z)
  float    v_iso;       // mm/s  máximo v_rms entre ejes (valor que se compara con la norma)
  float    temp_c;      // °C    temperatura interna del MPU6050 (contexto, no es ambiente)
  DlxAxisFeat ax[3];
  float    i_rms;       // A  RMS verdadero (incluye armónicos y ruido)
  float    i_pk;        // A  pico
  float    i_crest;     // i_pk / i_rms
  float    i1_rms;      // A  RMS de la fundamental (inmune al ruido de banda ancha)
  float    f_line;      // Hz frecuencia de red medida
  float    thd;         // distorsión armónica total (fracción, 0.05 = 5 %)
  float    bias_mv;     // mV offset de polarización del SCT013 (debería rondar 1650)
  uint32_t acc_missed;  // acumulados desde el arranque
  uint32_t adc_dropped;
  uint32_t net_dropped;
  float    dsp_ms;      // tiempo de cálculo de esta ventana
};

struct DlxInfo {
  char     fw[16];
  float    fs_acc_nom;
  float    fs_cur_nom;
  float    acc_ms2_per_lsb;  // a[m/s²] = cuenta * este factor
  float    amp_per_mv;       // i[A] = (mV - bias_mv) * este factor  (incluye CAL_FACTOR)
  uint16_t win_n;
  uint16_t adc_frame_n;
  float    iso_flo, iso_fhi;
  float    iso_ab, iso_bc, iso_cd;
  uint8_t  raw_mode;         // 0=off 1=continuo 2=ráfagas
  uint8_t  accel_range;      // 0..3 (±2,4,8,16 g)
  uint16_t raw_burst_s;
  uint16_t raw_period_s;
};

struct DlxSync {
  uint8_t pc_token[8];  // eco de lo que mandó la PC (su hora de envío)
  int64_t t_rx_us;      // hora ESP32 al recibir
  int64_t t_tx_us;      // hora ESP32 al responder
};

#pragma pack(pop)

static_assert(sizeof(DlxHdr) == 24, "DlxHdr");
static_assert(sizeof(DlxAccSample) == 10, "DlxAccSample");
static_assert(sizeof(DlxAxisFeat) == 40, "DlxAxisFeat");
