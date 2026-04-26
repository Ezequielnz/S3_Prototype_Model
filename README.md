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

## 🖧 Configuración para Múltiples Microcontroladores (Red LAN con W5500)

Si utilizas un switch de red no administrable (como el TP-Link TL-SG105) sin servidor de asignación automática de IPs (DHCP), debes realizar configuraciones únicas para cada nuevo ESP32-S3 que incorpores al sistema. 

**Antes de flashear el código en un nuevo microcontrolador, modifica en `src/main.cpp`:**

1. **Dirección MAC (`mac`):** Cada W5500 debe tener una placa MAC diferente.
   * *Ejemplo:* Incrementa el último dígito hexadecimal: `byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x02 };`
2. **Dirección IP del Dispositivo (`ip_local`):** Asigna una IP estática incrementando el último segmento.
   * *Ejemplo:* Si el MCU 1 tiene `192.168.0.101`, el MCU 2 debe llevar `192.168.0.102`.
3. **ID Lógico (`motor_id`):** Dentro de `sensor_task()`, este valor diferencia de forma lógica el dato inyectado al paquete.
   * *Ejemplo:* `current_payload.motor_id = 2;` 

Con este setup, puedes usar múltiples W5500 y empalmar los cables de todos al Switch. El hardware del switch se encarga de colisionar los paquetes UDP hacia el puerto final y el script Python decodificará sus variados `motor_id` a baja latencia.
