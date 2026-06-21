/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  DATALYX - EXPERIMENTO DE ALINEACIÓN DE FASE (GROUND TRUTH) ║
 * ║  Método: RAM Burst Logging (Cero Latencia de Escritura)     ║
 * ║  Sensores: MPU-6050 (I2C) y SCT-013 (ADC)                   ║
 * ╚══════════════════════════════════════════════════════════════╝
 */ 

#include <config.h>
#include <Arduino.h>
#include "calibration.h"

// --- CONFIGURACIÓN DEL EXPERIMENTO ---
const int TIEMPO_GRABACION_SEGUNDOS = 5;
const int FRECUENCIA_MUESTREO_HZ = 1000; // 1 kHz (1 muestra cada 1000 microsegundos)
const int INTERVALO_US = 1000000 / FRECUENCIA_MUESTREO_HZ;
const int TOTAL_MUESTRAS = TIEMPO_GRABACION_SEGUNDOS * FRECUENCIA_MUESTREO_HZ;

// --- ARREGLOS EN RAM (NUESTRO "DISCO DURO" DE ESTADO SÓLIDO) ---
int16_t buffer_accel_z[TOTAL_MUESTRAS];
uint16_t buffer_corriente[TOTAL_MUESTRAS];
uint32_t buffer_tiempo[TOTAL_MUESTRAS];

bool grabando = false;
bool datos_listos = false;

// Prototipo de la función de ráfaga
void ejecutarRafagaRAM();

void setup() {
    Serial.begin(2000000); // Máxima velocidad para descargar rápido
    
    // Configuración ADC para el Sensor de Corriente
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // Inicializar MPU6050 usando el módulo externo
    mpu_init();
    
    pinMode(PIN_INT, INPUT);

    Serial.println("\n--- DATALYX: SISTEMA LISTO ---");
    Serial.println("Envía la letra 'R' por el monitor serie para iniciar la ráfaga de grabación de 5 segundos.");
}

void loop() {
    // 1. ESPERAR LA ORDEN DE INICIO
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'R' || cmd == 'r') {
            grabando = true;
            datos_listos = false;
            Serial.println("¡GRABANDO EN RAM! ENCIENDE EL MOTOR AHORA...");
            ejecutarRafagaRAM();
        }
    }

    // 2. DESCARGAR DATOS CUANDO TERMINE
    if (datos_listos) {
        Serial.println("\n--- INICIO DESCARGA CSV ---");
        Serial.println("timestamp_us,accel_z_raw,corriente_raw"); // Cabecera del CSV
        
        for (int i = 0; i < TOTAL_MUESTRAS; i++) {
            Serial.print(buffer_tiempo[i]);
            Serial.print(",");
            Serial.print(buffer_accel_z[i]);
            Serial.print(",");
            Serial.println(buffer_corriente[i]);
        }
        
        Serial.println("--- FIN DESCARGA CSV ---");
        Serial.println("Copia este texto, guárdalo como .csv y ábrelo en tu Dashboard.");
        datos_listos = false; // Reset para un nuevo experimento
    }
}

// ==============================================================================
// FUNCIÓN CRÍTICA: BUCLE FÍSICO ESTRICTO
// ==============================================================================
void ejecutarRafagaRAM() {
    uint32_t inicio_experimento = esp_timer_get_time();
    
    for (int i = 0; i < TOTAL_MUESTRAS; i++) {
        uint32_t start_loop = esp_timer_get_time();
        
        // 1. ESTAMPADO DE TIEMPO EXACTO
        buffer_tiempo[i] = start_loop - inicio_experimento;

        // 2. LEER CORRIENTE (Muy rápido, crudo para procesar luego en PC)
        buffer_corriente[i] = analogRead(PIN_ADC);

        // 3. LEER VIBRACIÓN Z (Usando la función modularizada)
        mpu_read_accel_z(buffer_accel_z[i]);

        // 4. CONTROL DE TIEMPO ESTRICTO (Busy-Wait)
        // El MCU se queda aquí dando vueltas hasta que se cumplan exactamente los 1000us
        // Esto garantiza un muestreo sin Jitter.
        while (esp_timer_get_time() - start_loop < INTERVALO_US) {
            // No hacer nada, solo esperar el microsegundo exacto.
        }
    }
    
    grabando = false;
    datos_listos = true;
}
