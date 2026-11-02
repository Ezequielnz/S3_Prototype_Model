// Simulador de PC: corre la lógica REAL del firmware (extraída del .ino) con señales sintéticas
// de valores conocidos y envía los paquetes UDP a 127.0.0.1:5005 como lo haría el ESP32.
#include "sim_stubs.h"
#define FW_VERSION "sim"
#include <random>
#include <time.h>
#include "nodo_extraido.inc"

static int sock = -1;
static sockaddr_in dst;
void sim_enviar_paquete(const void *item) {
  const PaqueteRed *p = (const PaqueteRed *)item;
  sendto(sock, p->d, p->len, 0, (sockaddr *)&dst, sizeof(dst));
}

int main(int argc, char **argv) {
  double dur_s   = argc > 1 ? atof(argv[1]) : 25;      // duración simulada
  int    tiempoReal = argc > 2 ? atoi(argv[2]) : 1;    // 1 = ritmo real (para el receptor)
  const double fsA = 1003.0, fsC = 4001.6;             // relojes "reales" distintos de los nominales
  const double fl = 49.95;                             // frecuencia de red
  const double MMS = 1e-3 * sqrt(2.0);                 // mm/s RMS -> m/s pico
  std::mt19937 rng(1);
  std::normal_distribution<double> ruidoA(0, 0.05), ruidoI(0, 2.0);
  std::uniform_int_distribution<int> jitA(0, 4), jitC(2, 12);

  static SimCola cAcc{1, sizeof(MuestraAcc)}, cAdc{2, sizeof(TramaADC)}, cRed{3, sizeof(PaqueteRed)};
  qAcc = &cAcc; qAdc = &cAdc; qRed = &cRed;
  adcCanal = 2;
  for (int r = 0; r < 4096; r++) mvLut[r] = (uint16_t)(r * 3100 / 4095);
  fsAcc.fs = FS_ACC_NOM; fsCur.fs = FS_CUR_HZ;
  for (int e = 0; e < 3; e++) vel[e].iniciar(VEL_TD_FC, FS_ACC_NOM);

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  memset(&dst, 0, sizeof(dst));
  dst.sin_family = AF_INET; dst.sin_port = htons(PUERTO_PC); dst.sin_addr.s_addr = inet_addr("127.0.0.1");

  const bool sinADC = getenv("SIM_SIN_ADC"), hueco = getenv("SIM_HUECO");
  const double t0 = 1.0;
  long na = 0, nf = 0;
  double tInfo = -10;
  timespec ini; clock_gettime(CLOCK_MONOTONIC, &ini);
  for (;;) {
    double ta = t0 + na / fsA;
    double tc = t0 + (nf * ADC_FRAME_N + ADC_FRAME_N - 1) / fsC;
    double t = ta < tc ? ta : tc;
    if (t > t0 + dur_s) break;
    g_ahora_us = (int64_t)llround(t * 1e6);
    if (tiempoReal) {
      timespec ah; clock_gettime(CLOCK_MONOTONIC, &ah);
      double real = (ah.tv_sec - ini.tv_sec) + (ah.tv_nsec - ini.tv_nsec) * 1e-9;
      if (t - t0 > real) usleep((useconds_t)((t - t0 - real) * 1e6));
    }
    if (ta <= tc) {
      double ax = 2 * M_PI * 25.0 * 2.50 * MMS * sin(2 * M_PI * 25.0 * ta);
      double ay = 2 * M_PI * 2 * fl * 0.80 * MMS * sin(2 * M_PI * 2 * fl * ta)
                + 2 * M_PI * 5.0 * 3.00 * MMS * sin(2 * M_PI * 5.0 * ta);      // 5 Hz: fuera de la banda ISO
      double az = 9.80665 + 2 * M_PI * 157.0 * 0.50 * MMS * sin(2 * M_PI * 157.0 * ta) + ruidoA(rng);
      auto cnt = [](double a) { double c = llround(a / ESC_ACC); return (int16_t)(c > 32767 ? 32767 : c < -32768 ? -32768 : c); };
      MuestraAcc s;
      s.t = (int64_t)llround(ta * 1e6) + jitA(rng);
      s.x = cnt(ax); s.y = cnt(ay); s.z = cnt(az);
      s.temp = (int16_t)((31.0 - 36.53) * 340);
      if (!(hueco && na >= 5000 && na < 5005)) procesarAcc(s);
      na++;
    } else {
      TramaADC f;
      for (int i = 0; i < ADC_FRAME_N; i++) {
        double ti = t0 + (nf * ADC_FRAME_N + i) / fsC;
        double w = 2 * M_PI * fl * ti;
        double iA = sqrt(2.0) * 10.0 * (sin(w) + 0.04 * sin(5 * w + 0.3) + 0.03 * sin(7 * w + 1.1));
        double mv = 1650 + iA / AMP_POR_MV + ruidoI(rng);
        int r = (int)llround(mv / 3100.0 * 4095);
        f.raw[i] = (uint16_t)(r < 0 ? 0 : r > 4095 ? 4095 : r);
      }
      f.n = ADC_FRAME_N;
      f.tUltima = (int64_t)llround(tc * 1e6) + jitC(rng);
      if (!sinADC) procesarTramaADC(f);
      nf++;
    }
    // Lo mismo que hace tareaDSP en cada vuelta
    if (vPendiente >= 0) {
      Ventana &w = ven[vPendiente];
      if (corrienteCubre(w.tUlt + (int64_t)(1e6f / fsAcc.fs))) { calcularVentana(w); vPendiente = -1; }
    }
    if (t - tInfo > 5) { tInfo = t; enviarInfo(); }
  }
  return 0;
}
