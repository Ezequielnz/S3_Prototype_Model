/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  PREPROCESADOR DE ALTA VELOCIDAD — ESP32-S3 + MPU6050       ║
 * ║  Tasa: 1kHz  |  Algoritmo Usuario |  Salida: UART           ║
 * ╚══════════════════════════════════════════════════════════════╝
 */ 

#include <config.h>
#include <Arduino.h>
#include <Wire.h>
/*
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"

#include "datalyx_payload.h"
#include "vibration_analyzer.h"
*/

// ═══════════════════════════════════════════════════════════════
//  CONFIGURACIÓN HARDWARE Y RED (W5500)
// ═══════════════════════════════════════════════════════════════
#define MPU_ADDR          0x68
#define PIN_SDA           8
#define PIN_SCL           9

#define PIN_INT           6

/*

// Pines SPI para el W5500 en el S3 Supermini
#define W5500_CS_PIN      10
#define W5500_SCK_PIN     12
#define W5500_MOSI_PIN    11
#define W5500_MISO_PIN    13
#define W5500_RST_PIN     5

// Configuración de RED (Asignar IPs estáticas ya que no hay Router DHCP)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 }; // Única por ESP32
IPAddress ip_local(10, 0, 0, 101);                   // IP del ESP32 (Subred aislada)
IPAddress ip_destino(10, 0, 0, 100);                 // IP de la PC u Orange Pi (Subred aislada)
unsigned int localPort = 8888;
unsigned int destPort  = 5000;

EthernetUDP Udp;
*/

// MPU6050 Registros
#define REG_PWR_MGMT_1    0x6B
#define REG_SMPLRT_DIV    0x19
/*
#define REG_CONFIG        0x1A
*/
#define REG_ACCEL_CONFIG  0x1C
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_ACCEL_XOUT_H  0x3B
#define REG_ACCEL_ZOUT_H  0x3F

unsigned long tiempoAnterior = 0;
const unsigned long intervalo = 4167;

/*
QueueHandle_t packet_queue;
constexpr uint8_t QUEUE_SIZE = 4; // Buffer para 4 paquetes DatalyxPayload completos

TaskHandle_t sensor_task_handle = NULL;

IRAM_ATTR void mpu_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
*/

IRAM_ATTR void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}

IRAM_ATTR bool mpu_read_accel_z(int16_t &z) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_ZOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true) != 2) return false;
    z = (Wire.read() << 8) | Wire.read();
    return true;
}

IRAM_ATTR bool mpu_read_accel(int16_t &x, int16_t &y, int16_t &z) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true) != 6) return false;
    x = (Wire.read() << 8) | Wire.read();
    y = (Wire.read() << 8) | Wire.read();
    z = (Wire.read() << 8) | Wire.read();
    return true;
}

/*
void mpu_clear_int() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3A); // Registro INT_STATUS (leerlo limpia el pin INT)
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (bool)true);
    Wire.read();
}
*/

void mpu_init() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(1000000);  // 1 MHz — Fast Mode+
    Wire.setTimeOut(10);     // Timeout 10ms anti-cuelgues por ruido eléctrico

    // Verificación de conexión I2C (WHO_AM_I) - Comprobación de cables
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
        uint8_t whoami = Wire.read();
        if (whoami == 0x68) {
            Serial.println("[MPU] Sensor detectado correctamente!");
        } else {
            Serial.printf("[MPU] ATENCIÓN: Dispositivo detectado pero WHO_AM_I es 0x%02X (Debería ser 0x68)\n", whoami);
        }
    } else {
        Serial.println("[ERROR CRÍTICO] MPU6050 NO ENCONTRADO. ¡Revisa que SDA esté en el pin 8, SCL en el pin 9, VCC y GND!");
    }

    mpu_write(REG_PWR_MGMT_1,  0x01);  // Despertar
    // delay(100); // Comentado para no tener delay

    mpu_write(REG_SMPLRT_DIV,  0x00);  // Rate = 1kHz
    /*
    mpu_write(REG_CONFIG,      0x02);  // DLPF 94Hz (Filtro comentado)
    */
    mpu_write(REG_ACCEL_CONFIG, 0x08); // ±4g: 8192 LSB/g
    mpu_write(REG_INT_PIN_CFG, 0x10);  // INT Active High
    mpu_write(REG_INT_ENABLE,  0x01);  // DATA_RDY_EN

    Serial.println("[MPU] Inicializado: 1kHz");
}

/*
void sensor_task(void* pvParameters) {
    VibrationAnalyzer analyzer;
    const float SCALE = 1.0f / 8192.0f; // Pasar de LSB a G (±4g)
    
    DatalyxPayload current_payload;
    int sample_index = 0;
    
    while (true) {
        // Esperar eficientemente la interrupción, con timeout de 5ms por si el pin INT se pega
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5)) == 0) {
            // Timeout: El pin INT quedó pegado en HIGH o hubo cuelgue I2C.
            // Reiniciamos el periférico I2C del ESP32 por hardware
            Wire.end(); 
            Wire.begin(PIN_SDA, PIN_SCL);
            Wire.setClock(1000000);
            Wire.setTimeOut(10);
            mpu_clear_int();
            continue;
        }
        
        int16_t rz;
        if (!mpu_read_accel_z(rz)) {
            // Falla de lectura por ruido I2C.
            Wire.end(); 
            Wire.begin(PIN_SDA, PIN_SCL);
            Wire.setClock(1000000);
            Wire.setTimeOut(10);
            mpu_clear_int();
            continue;
        }
        
        // El sensor lee Z, escalar de LSB a G
        float az_g = (float)rz * SCALE;
        
        // Calcular la velocidad
        float velocidad = analyzer.calcularVelocidadVibracion(az_g);
        
        current_payload.velocity_buffer[sample_index] = velocidad;
        sample_index++;
        
        if (sample_index >= SAMPLES_PER_PAYLOAD) {
            // Calcular métricas RMS y Pico-Pico
            float sum_sq = 0.0f;
            float max_v = -10000.0f;
            float min_v = 10000.0f;
            
            for (int i = 0; i < SAMPLES_PER_PAYLOAD; i++) {
                float v = current_payload.velocity_buffer[i];
                sum_sq += v * v;
                if (v > max_v) max_v = v;
                if (v < min_v) min_v = v;
            }
            
            current_payload.rms_velocity = sqrtf(sum_sq / (float)SAMPLES_PER_PAYLOAD);
            current_payload.peak_to_peak = max_v - min_v;
            
            current_payload.timestamp_ms = millis();
            current_payload.motor_id = 1;
            
            // Checksum ignorando el propio campo checksum
            current_payload.checksum = calcularChecksum(&current_payload, sizeof(DatalyxPayload) - 2);
            
            xQueueSend(packet_queue, &current_payload, 0);
            sample_index = 0;
        }
    }
}

void comms_task(void* pvParameters) {
    // Reset por hardware del W5500 para mayor estabilidad
    pinMode(W5500_RST_PIN, OUTPUT);
    digitalWrite(W5500_RST_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(W5500_RST_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(150));

    // Configurar los pines SPI específicos (SCK, MISO, MOSI, CS)
    SPI.begin(W5500_SCK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN, W5500_CS_PIN);

    // Inicializar el chip W5500
    Ethernet.init(W5500_CS_PIN);
    
    // Iniciar conexión con IP Estática
    Ethernet.begin(mac, ip_local);
    
    // Esperar a que el link de hardware se establezca
    while (true) {
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
            Serial.println("[Red] Error: W5500 no detectado (Fallo SPI). Revisa los cables SCK, MISO, MOSI, CS.");
        } else if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println("[Red] W5500 detectado (SPI OK), pero no hay Link (Cable desconectado o no Auto-MDIX).");
        } else {
            Serial.println("[Red] Hardware y Link OK!");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    Udp.begin(localPort);
    Serial.println("[Red] W5500 Iniciado por UDP. Enviando payloads.");

    DatalyxPayload pkt;

    while (true) {
        if (xQueueReceive(packet_queue, &pkt, portMAX_DELAY) == pdTRUE) {
            // Enviar los 1040 bytes por UDP
            Udp.beginPacket(ip_destino, destPort);
            Udp.write((const uint8_t*)&pkt, sizeof(DatalyxPayload));
            Udp.endPacket();
            
            static int pkt_count = 0;
            pkt_count++;
            if (pkt_count % 4 == 0) { // Imprimir 1 vez por segundo aprox (4 payloads de 256 ms)
                Serial.println("[W5500] Paquete UDP de 1040 bytes enviado a la PC!");
            }
        }
    }
}

#define LED_BUILTIN_PIN 47 // Pin del LED azul en el S3 Super Mini
*/

void setup() {
    Serial.begin(115200);
    //setCpuFrequencyMhz(240);

    /*
    // Configurar el LED interno para que sepamos que está encendido
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, LOW); // En muchos S3 el LED enciende en estado LOW
    */

    mpu_init();
    
    pinMode(PIN_INT, INPUT);

    /*
    packet_queue = xQueueCreate(QUEUE_SIZE, sizeof(DatalyxPayload));

    pinMode(PIN_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_INT), mpu_isr, RISING);

    // Ajustar stack size a 8192 porque DatalyxPayload ocupa ~1KB y está en stack local
    xTaskCreatePinnedToCore(sensor_task, "sensor", 8192, nullptr, 24, &sensor_task_handle, 1);
    xTaskCreatePinnedToCore(comms_task, "comms", 8192, nullptr, 20, nullptr, 0);
    */
}

void loop() {
    /*
    // Parpadeo de "Latido de corazón" (Heartbeat)
    digitalWrite(LED_BUILTIN_PIN, LOW);  // Enciende
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(LED_BUILTIN_PIN, HIGH); // Apaga
    vTaskDelay(pdMS_TO_TICKS(900));
    */
    
    unsigned long tiempoActual = micros();
    
    if (tiempoActual - tiempoAnterior >= intervalo) {
        tiempoAnterior = tiempoActual;

        int16_t rx, ry, rz;
        if (mpu_read_accel(rx, ry, rz)) {
            float ax_g = (float)rx * (1.0f / 8192.0f);
            float ay_g = (float)ry * (1.0f / 8192.0f);
            float az_g = (float)rz * (1.0f / 8192.0f);
            
            Serial.println(ax_g);
        }
    }
}
