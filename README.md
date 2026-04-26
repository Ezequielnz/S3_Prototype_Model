# S3 Prototype Model (ESP32-S3)

Este proyecto maneja la adquisición y el pre-procesamiento de señales de vibración obtenidas mediante un sensor MPU6050 a una frecuencia exacta de muestreo de 1000 Hz. Utiliza una arquitectura basada en FreeRTOS para el ESP32-S3 diseñando separar la tarea de sensado (Core 1) de la de comunicación (Core 0).

## Análisis de Vibración (DSP)

El microcontrolador realiza un cálculo matemático en tiempo real para transformar la aceleración bruta (medida sobre el Eje Z) en velocidad instantánea de vibración:
1. **Factor de Escala:** Se escalan los Gs del acelerómetro a mm/s².
2. **Filtro Paso-Alto (DC Blocker):** Método ultrarrápido para suprimir el vector de la gravedad y sesgos del sensor.
3. **Integración con Fuga (Leaky Integrator):** Mediante la regla del trapecio, integramos la aceleración pura y estable en velocidad.

La lógica núcleo ha sido modularizada bajo el módulo `vibration_analyzer.cpp`.

## Protocolo Datalyx Payload

Dado el requerimiento computacional del procesador destino para la inferencia con AI, los datos son empaquetados en ráfagas de 256 muestras a través del modelo en C++ `DatalyxPayload`.
El tamaño constante por trampa es equivalente a **1040 bytes** sin espacios muertos de memoria (`__attribute__((packed))`).

### Estructura
| Segmento            | Tamaño | Tipo de Dato (C++)   | Descripción                            |
|---------------------|--------|----------------------|----------------------------------------|
| **Metadatos**       | 4 B    | `uint32_t`           | Timestamp en ms                        |
| **Metadatos**       | 2 B    | `uint16_t`           | Motor / Nodo ID                        |
| **Estadísticas**    | 4 B    | `float`              | Velocidad media cuadrática (RMS)       |
| **Estadísticas**    | 4 B    | `float`              | Amplitud Pico a Pico (Pk-Pk)           |
| **Datos Crudos**    | 1024 B | `float[256]`         | Arreglo completo con las ondas DSP     |
| **Seguridad EMI**   | 2 B    | `uint16_t`           | Suma de verificación simple (Checksum) |

Los microcontroladores envían estas tramas de 1040 bytes de manera constante utilizando enlace Serial (UART), logrando inyecciones de un paquete cada ~256 milisegundos hacia Orange Pi.
