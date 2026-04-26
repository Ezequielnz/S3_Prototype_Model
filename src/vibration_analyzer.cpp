#include "vibration_analyzer.h"

VibrationAnalyzer::VibrationAnalyzer() {
    reset();
}

void VibrationAnalyzer::reset() {
    a_raw_prev = 0.0f;
    a_filt_prev = 0.0f;
    v_prev = 0.0f;
}

// Esta función es tu núcleo de procesamiento matemático. 
// Debe llamarse CADA VEZ que el MPU6050 tenga un dato listo (preferiblemente vía Interrupción).
float VibrationAnalyzer::calcularVelocidadVibracion(float aceleracion_en_G) {
    // 1. Escalar de G a mm/s^2
    float a_raw = aceleracion_en_G * G_TO_MMS2;

    // 2. Filtro Paso-Alto (DC Blocker) ultrarrápido - Quita la gravedad y el bias
    float a_filt = a_raw - a_raw_prev + (R_DC_BLOCKER * a_filt_prev);

    // 3. Integración Numérica (Regla del Trapecio) + Integrador con Fuga (Leaky)
    float velocidad_mms = (GAMMA_LEAKY * v_prev) + ((a_filt + a_filt_prev) * 0.5f * DT_ALG);

    // 4. Actualizar variables de estado para el siguiente ciclo de reloj
    a_raw_prev = a_raw;
    a_filt_prev = a_filt;
    v_prev = velocidad_mms;

    // El resultado es la velocidad instantánea de vibración en mm/s, centrada en cero.
    return velocidad_mms;
}
