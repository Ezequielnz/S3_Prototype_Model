#ifndef DATALYX_PAYLOAD_H
#define DATALYX_PAYLOAD_H

#include <stdint.h>
#include <stddef.h>

#define SAMPLES_PER_PAYLOAD 256 

struct __attribute__((packed)) DatalyxPayload {
    // --- METADATOS (Cabecera) ---
    uint32_t timestamp_ms;       // 4 bytes: Marca de tiempo
    uint16_t motor_id;           // 2 bytes: ID del nodo/motor
    
    // --- MÉTRICAS GLOBALES ---
    float rms_velocity;          // 4 bytes: Valor RMS en mm/s
    float peak_to_peak;          // 4 bytes: Valor Pico a Pico en mm/s
    
    // --- LOS DATOS CRUDOS PROCESADOS ---
    float velocity_buffer[SAMPLES_PER_PAYLOAD]; // 1024 bytes: Onda de velocidad pura
    
    // --- SEGURIDAD DE TRANSMISIÓN ---
    uint16_t checksum;           // 2 bytes: Suma de verificación contra ruido eléctrico (EMI)
};

// Función para calcular checksum
inline uint16_t calcularChecksum(const void* data, size_t length) {
    const uint8_t* bytes = (const uint8_t*)data;
    uint16_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum += bytes[i];
    }
    return checksum;
}

#endif // DATALYX_PAYLOAD_H
