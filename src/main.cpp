/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  PREPROCESADOR DE ALTA VELOCIDAD — ESP32-S3 + MPU6050       ║
 * ║  Tasa: 1kHz  |  Algoritmo Usuario |  Salida: UART           ║
 * ╚══════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"

#include "datalyx_payload.h"
#include "vibration_analyzer.h"

// ═══════════════════════════════════════════════════════════════
//  CONFIGURACIÓN HARDWARE Y RED (W5500)
// ═══════════════════════════════════════════════════════════════
#define MPU_ADDR          0x68
#define PIN_SDA           8
#define PIN_SCL           9
#define PIN_INT           4

// Pines SPI para el W5500 en el S3 Supermini
#define W5500_CS_PIN      10
#define W5500_SCK_PIN     12
#define W5500_MOSI_PIN    11
#define W5500_MISO_PIN    13
#define W5500_RST_PIN     5

// Configuración de RED (Asignar IPs estáticas ya que no hay Router DHCP)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 }; // Única por ESP32
IPAddress ip_local(192, 168, 0, 101);                // IP del ESP32
IPAddress ip_destino(192, 168, 0, 100);              // IP de la PC u Orange Pi
unsigned int localPort = 8888;
unsigned int destPort  = 5000;

EthernetUDP Udp;

// MPU6050 Registros
#define REG_PWR_MGMT_1    0x6B
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_ACCEL_CONFIG  0x1C
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_ACCEL_ZOUT_H  0x3F

QueueHandle_t packet_queue;
constexpr uint8_t QUEUE_SIZE = 4; // Buffer para 4 paquetes DatalyxPayload completos

volatile bool isr_flag = false;

IRAM_ATTR void mpu_isr() {
    isr_flag = true;
}

IRAM_ATTR void mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}

IRAM_ATTR bool mpu_read_accel_z(int16_t &z) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_ZOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (bool)true) != 2) return false;
    z = (Wire.read() << 8) | Wire.read();
    return true;
}

void mpu_init() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(1000000);  // 1 MHz — Fast Mode+

    mpu_write(REG_PWR_MGMT_1,  0x01);  // Despertar
    delay(100);

    mpu_write(REG_SMPLRT_DIV,  0x00);  // Rate = 1kHz
    mpu_write(REG_CONFIG,      0x02);  // DLPF 94Hz
    mpu_write(REG_ACCEL_CONFIG, 0x08); // ±4g: 8192 LSB/g
    mpu_write(REG_INT_PIN_CFG, 0x10);  // INT Active High
    mpu_write(REG_INT_ENABLE,  0x01);  // DATA_RDY_EN

    Serial.println("[MPU] Inicializado: 1kHz");
}

void sensor_task(void* pvParameters) {
    VibrationAnalyzer analyzer;
    const float SCALE = 1.0f / 8192.0f; // Pasar de LSB a G (±4g)
    
    DatalyxPayload current_payload;
    int sample_index = 0;
    
    while (true) {
        if (!isr_flag) {
            taskYIELD();
            continue;
        }
        isr_flag = false;
        
        int16_t rz;
        if (!mpu_read_accel_z(rz)) continue;
        
        // El sensor lee Z, escalar de LSB a G
        float az_g = (float)rz * SCALE;
        
        // Calcular la velocidad usando el algoritmo provisto por el usuario
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
    while (Ethernet.hardwareStatus() == EthernetNoHardware || Ethernet.linkStatus() == LinkOFF) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.println("[Red] Esperando conexión al Switch...");
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
        }
    }
}

void setup() {
    Serial.begin(115200);
    setCpuFrequencyMhz(240);

    mpu_init();
    packet_queue = xQueueCreate(QUEUE_SIZE, sizeof(DatalyxPayload));

    pinMode(PIN_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_INT), mpu_isr, RISING);

    // Ajustar stack size a 8192 porque DatalyxPayload ocupa ~1KB y está en stack local
    xTaskCreatePinnedToCore(sensor_task, "sensor", 8192, nullptr, 24, nullptr, 1);
    xTaskCreatePinnedToCore(comms_task, "comms", 8192, nullptr, 20, nullptr, 0);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
