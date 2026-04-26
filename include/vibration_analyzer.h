#ifndef VIBRATION_ANALYZER_H
#define VIBRATION_ANALYZER_H

#include <math.h>

// --- PARÁMETROS MATEMÁTICOS DEL SISTEMA ---
constexpr float FS = 1000.0f;               // Frecuencia de muestreo (1000 Hz)
constexpr float DT_ALG = 1.0f / FS;         // Delta de tiempo (0.001 s)
constexpr float G_TO_MMS2 = 9806.65f;       // Conversión de G a mm/s^2

// --- PARÁMETROS DE LOS FILTROS ---
// R = 1 - (2 * PI * f_corte / Fs). Para Fcorte = 5 Hz y Fs = 1000 Hz -> R ≈ 0.968
constexpr float R_DC_BLOCKER = 0.968f;      
// Gamma para el integrador con fuga (Leaky Integrator). Muy cercano a 1.
constexpr float GAMMA_LEAKY = 0.995f;       

class VibrationAnalyzer {
private:
    float a_raw_prev;
    float a_filt_prev;
    float v_prev;

public:
    VibrationAnalyzer();
    void reset();
    float calcularVelocidadVibracion(float aceleracion_en_G);
};

#endif // VIBRATION_ANALYZER_H
