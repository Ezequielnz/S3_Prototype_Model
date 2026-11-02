/*
 * dlx_dsp.h — Procesamiento de señal del nodo Datalyx
 *
 * Funciones puras (sin hardware), así se pueden probar en una PC con señales sintéticas.
 * En el ESP32 usa la FFT optimizada de esp-dsp (instrucciones SIMD del S3);
 * en la PC usa una FFT radix-2 propia equivalente.
 *
 * Convención de escalado de la FFT (ventana Hann, posible relleno con ceros):
 *   P[k] = 2 |X[k]|² / (NFFT · Σw²)   ->   Σ_k P[k] ≈ RMS² de la señal
 *   (teorema de Parseval corregido por la energía de la ventana)
 *   Así la RMS en cualquier banda es sqrt(Σ P[k] en la banda).
 */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "dlx_proto.h"

#ifdef ESP_PLATFORM
  #include "esp_dsp.h"
#endif

namespace dlx {

static const float DOS_PI = 6.28318530718f;

constexpr int potencia2(int n) { return n <= 1 ? 1 : 2 * potencia2((n + 1) / 2); }

// ============================ FFT ===================================
// d: NFFT números complejos intercalados [re0, im0, re1, im1, ...], alineado a 16 bytes.
inline bool fftInit(int nMax) {
#ifdef ESP_PLATFORM
  return dsps_fft2r_init_fc32(NULL, nMax) == ESP_OK;
#else
  (void)nMax;
  return true;
#endif
}

inline void fft(float *d, int N) {
#ifdef ESP_PLATFORM
  dsps_fft2r_fc32(d, N);
  dsps_bit_rev_fc32(d, N);
#else
  for (int i = 1, j = 0; i < N; i++) {             // reordenamiento bit-reverso
    int bit = N >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float tr = d[2*i], ti = d[2*i+1];
      d[2*i] = d[2*j]; d[2*i+1] = d[2*j+1];
      d[2*j] = tr;     d[2*j+1] = ti;
    }
  }
  for (int len = 2; len <= N; len <<= 1) {          // mariposas
    double ang = -2.0 * M_PI / len;
    for (int i = 0; i < N; i += len) {
      for (int k = 0; k < len / 2; k++) {
        double wr = cos(ang * k), wi = sin(ang * k);
        int a = i + k, b = i + k + len / 2;
        double xr = d[2*b] * wr - d[2*b+1] * wi;
        double xi = d[2*b] * wi + d[2*b+1] * wr;
        d[2*b]   = (float)(d[2*a]   - xr);
        d[2*b+1] = (float)(d[2*a+1] - xi);
        d[2*a]   = (float)(d[2*a]   + xr);
        d[2*a+1] = (float)(d[2*a+1] + xi);
      }
    }
  }
#endif
}

// Carga x (sin su media) con ventana Hann en work, rellena con ceros hasta NFFT,
// hace la FFT y deja en work[0..NFFT/2] la potencia escalada P[k] (ver encabezado).
// Devuelve la media de x.
inline float espectroPotencia(const float *x, int M, int NFFT, float *work) {
  if (M > NFFT) M = NFFT;
  double s = 0;
  for (int i = 0; i < M; i++) s += x[i];
  float media = (float)(s / M);
  double sw2 = 0;
  for (int n = 0; n < M; n++) {
    float w = 0.5f - 0.5f * cosf(DOS_PI * n / (M - 1));
    work[2*n]   = (x[n] - media) * w;
    work[2*n+1] = 0.0f;
    sw2 += (double)w * w;
  }
  for (int n = M; n < NFFT; n++) { work[2*n] = 0.0f; work[2*n+1] = 0.0f; }
  fft(work, NFFT);
  float esc = (float)(2.0 / ((double)NFFT * sw2));
  // Compactación in-place: work[k] = P[k]. Seguro porque k <= 2k (nunca pisa lo que falta leer).
  for (int k = 0; k <= NFFT / 2; k++) {
    float re = work[2*k], im = work[2*k+1];
    work[k] = (re * re + im * im) * esc;
  }
  work[0] *= 0.5f;          // DC y Nyquist no se duplican
  work[NFFT / 2] *= 0.5f;
  return media;
}

// Interpolación parabólica sobre ln(P) alrededor de un máximo (Hann ≈ gaussiana en log).
// Devuelve el corrimiento en bins, entre -0.5 y 0.5.
inline float interpPico(float pm1, float p0, float pp1) {
  if (pm1 <= 0 || p0 <= 0 || pp1 <= 0) return 0.0f;
  float a = logf(pm1), b = logf(p0), c = logf(pp1);
  float den = a - 2.0f * b + c;
  if (fabsf(den) < 1e-12f) return 0.0f;
  float d = 0.5f * (a - c) / den;
  if (d > 0.5f) d = 0.5f;
  if (d < -0.5f) d = -0.5f;
  return d;
}

// Energía de un tono: suma P en ±2 bins alrededor de kc (lóbulo principal de Hann).
inline float energiaTono(const float *P, int kc, int kMax) {
  float e = 0;
  for (int k = kc - 2; k <= kc + 2; k++)
    if (k >= 1 && k <= kMax) e += P[k];
  return e;
}

// ===================== FILTROS TEMPORALES ===========================
// Biquad pasa-altos Butterworth de 2º orden (transformada bilineal), forma directa II transpuesta.
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  void pasaAltos(float fc, float fs) {
    const float Q = 0.70710678f;
    float K = tanf((float)M_PI * fc / fs);
    float nrm = 1.0f / (1.0f + K / Q + K * K);
    b0 = nrm; b1 = -2.0f * nrm; b2 = nrm;
    a1 = 2.0f * (K * K - 1.0f) * nrm;
    a2 = (1.0f - K / Q + K * K) * nrm;
    z1 = z2 = 0;
  }
  inline float paso(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

// Cadena aceleración -> velocidad en el dominio del tiempo (como un vibrómetro ISO 2954):
//   pasa-altos 2º orden -> integración trapezoidal (con fuga mínima a 0.1 Hz) -> pasa-altos 2º orden
// Con fc = 4 Hz el error a 10 Hz es ≈ -2.5 %, despreciable desde 15 Hz.
struct CadenaVelocidad {
  Biquad h1, h2;
  float vInt = 0, xAnt = 0, fuga = 1, dt = 0.001f;
  void iniciar(float fc, float fs) {
    h1.pasaAltos(fc, fs);
    h2.pasaAltos(fc, fs);
    dt = 1.0f / fs;
    fuga = 1.0f - DOS_PI * 0.1f * dt;
    vInt = xAnt = 0;
  }
  void ajustarFs(float fs) {           // corrige la ganancia del integrador con la fs medida
    dt = 1.0f / fs;
    fuga = 1.0f - DOS_PI * 0.1f * dt;
  }
  // Recibe la primera muestra para arrancar sin el escalón de la gravedad.
  void precargar(float a0) {
    // Estado estacionario de un pasa-altos ante entrada constante a0: salida 0.
    // DF2T: y = b0 x + z1 = 0  ->  z1 = -b0 a0 ; z2 = b2 a0 - a2*0 = b2 a0
    h1.z1 = -h1.b0 * a0;
    h1.z2 = h1.b2 * a0;
  }
  inline float paso(float a) {         // a en m/s² -> devuelve m/s
    float x = h1.paso(a);
    vInt = fuga * vInt + 0.5f * (x + xAnt) * dt;
    xAnt = x;
    return h2.paso(vInt);
  }
};

// ====================== INDICADORES POR EJE =========================
// a: aceleración del eje en m/s² (puede incluir gravedad), M muestras.
// work: 2*NFFT floats alineados a 16 bytes. fLinea: frecuencia de red (0 si se desconoce).
inline void indicadoresEje(const float *a, int M, int NFFT, float fs,
                           float fLo, float fHi, float fLinea,
                           float *work, DlxAxisFeat &o) {
  // --- Estadísticos temporales (sobre la parte vibratoria) ---
  double s = 0;
  for (int i = 0; i < M; i++) s += a[i];
  float media = (float)(s / M);
  double s2 = 0, s4 = 0;
  float pk = 0;
  for (int i = 0; i < M; i++) {
    float x = a[i] - media;
    double x2 = (double)x * x;
    s2 += x2; s4 += x2 * x2;
    if (fabsf(x) > pk) pk = fabsf(x);
  }
  double m2 = s2 / M;
  o.a_rms = (float)sqrt(m2);
  o.a_pk  = pk;
  o.crest = o.a_rms > 1e-9f ? pk / o.a_rms : 0.0f;
  o.kurt  = m2 > 1e-18 ? (float)((s4 / M) / (m2 * m2)) : 0.0f;

  // --- Espectro de velocidad: V(f) = A(f) / (2πf) ---
  espectroPotencia(a, M, NFFT, work);
  const float *P = work;
  const int kMax = NFFT / 2 - 1;
  float df = fs / NFFT;
  int k0 = (int)ceilf(fLo / df);  if (k0 < 1) k0 = 1;
  int k1 = (int)floorf(fHi / df); if (k1 > kMax) k1 = kMax;

  double sv = 0;
  int kp = -1;
  float pvMax = 0;
  for (int k = k0; k <= k1; k++) {
    float w = DOS_PI * k * df;
    float pv = P[k] / (w * w);
    sv += pv;
    if (pv > pvMax) { pvMax = pv; kp = k; }
  }
  o.v_rms = (float)sqrt(sv) * 1000.0f;   // m/s -> mm/s

  // --- Componente dominante (frecuencia interpolada, amplitud por energía del lóbulo) ---
  if (kp > 1 && kp < kMax) {
    auto pvk = [&](int k) { float w = DOS_PI * k * df; return P[k] / (w * w); };
    float d = interpPico(pvk(kp - 1), pvk(kp), pvk(kp + 1));
    o.f_dom = (kp + d) * df;
    float w = DOS_PI * o.f_dom;
    o.v_dom = sqrtf(energiaTono(P, kp, kMax)) / w * 1000.0f;
  } else {
    o.f_dom = 0; o.v_dom = 0;
  }

  // --- Componente a 2 x f_red (fuerza magnética del motor) ---
  o.v_2fl = 0;
  if (fLinea > 30.0f) {
    float f2 = 2.0f * fLinea;
    int kc = (int)lroundf(f2 / df);
    if (kc - 2 >= 1 && kc + 2 <= kMax) {
      float w = DOS_PI * f2;
      o.v_2fl = sqrtf(energiaTono(P, kc, kMax)) / w * 1000.0f;
    }
  }
}

// ===================== INDICADORES DE CORRIENTE =====================
struct IndCorriente {
  float i_rms, i_pk, i_crest, i1_rms, f_line, thd, bias_mv;
};

// mv: muestras del ADC en mV, M muestras a fs. ampPorMv: A por mV (incluye CAL_FACTOR).
inline void indicadoresCorriente(const float *mv, int M, int NFFT, float fs, float ampPorMv,
                                 float *work, IndCorriente &o) {
  memset(&o, 0, sizeof(o));
  if (M < 64) return;
  double s = 0;
  for (int i = 0; i < M; i++) s += mv[i];
  float bias = (float)(s / M);
  double s2 = 0;
  float pk = 0;
  for (int i = 0; i < M; i++) {
    float x = (mv[i] - bias) * ampPorMv;
    s2 += (double)x * x;
    if (fabsf(x) > pk) pk = fabsf(x);
  }
  o.bias_mv = bias;
  o.i_rms   = (float)sqrt(s2 / M);
  o.i_pk    = pk;
  o.i_crest = o.i_rms > 1e-6f ? pk / o.i_rms : 0.0f;

  espectroPotencia(mv, M, NFFT, work);      // potencia en mV²
  const float *P = work;
  const int kMax = NFFT / 2 - 1;
  float df = fs / NFFT;
  float a2 = ampPorMv * ampPorMv;

  // Fundamental: máximo entre 40 y 70 Hz (cubre redes de 50 y 60 Hz y variadores lentos no)
  int ka = (int)ceilf(40.0f / df), kb = (int)floorf(70.0f / df);
  int k1 = ka;
  for (int k = ka; k <= kb; k++) if (P[k] > P[k1]) k1 = k;
  float d = interpPico(P[k1 - 1], P[k1], P[k1 + 1]);
  float f1 = (k1 + d) * df;
  float e1 = energiaTono(P, k1, kMax) * a2;
  o.i1_rms = sqrtf(e1);
  if (o.i1_rms < 1e-4f) return;              // sin corriente: no hay fundamental que medir
  o.f_line = f1;

  // THD: armónicos 2..40 hasta Nyquist (ventana de ±2 bins centrada en h·f1)
  double eh = 0;
  for (int h = 2; h <= 40; h++) {
    int kc = (int)lroundf(h * f1 / df);
    if (kc + 2 > kMax) break;
    eh += energiaTono(P, kc, kMax) * a2;
  }
  o.thd = (float)(sqrt(eh) / o.i1_rms);
}

// Zona ISO a partir de v_rms (mm/s) y los tres límites A/B, B/C, C/D.
inline uint8_t zonaISO(float v, float ab, float bc, float cd) {
  return v < ab ? 0 : v < bc ? 1 : v < cd ? 2 : 3;
}

}  // namespace dlx
