#include <Arduino.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"
#include "ISO_Analyzer.h"     // Your ISO 20816 vibration analysis library
#include "NetworkManager.h"   // The corrected NetworkManager library

// --- Global Object Instantiation ---
ISO_Analyzer analyzer;
NetworkManager network(WIFI_SSID, WIFI_PASS, MQTT_BROKER_IP, MQTT_PORT);
Adafruit_MPU6050 mpu;

/**
 * @brief Callback function triggered when a new JSON payload arrives via MQTT.
 * * In an industrial environment, when the central Python server sends a new 
 * configuration (e.g., new machine limits, new sampling rates, or a new name),
 * this function saves the state to non-volatile memory so the device remembers
 * it after a power loss.
 * * @param doc The parsed JSON document received from the MQTT broker.
 */
void onConfigurationReceived(JsonDocument& doc) {
    Serial.println("\n[SYSTEM] Applying new configuration from PC...");
    
    // 1. Open LittleFS in write mode to save the new configuration
    File file = LittleFS.open(CONFIG_FILE_PATH, "w");
    if (file) {
        serializeJson(doc, file);
        file.close();
        Serial.println("[SYSTEM] New config saved to LittleFS.");
        
        // 2. Reload the analyzer limits/parameters with the new data
        if (analyzer.loadConfig(CONFIG_FILE_PATH)) {
            
            // 3. Check if the server assigned a new specific name to this machine
            // If the name changed, the NetworkManager will disconnect and reconnect
            // with the new MQTT topics automatically.
            String newMachineName = analyzer.getConfig().machineName;
            network.updateDeviceId(newMachineName);
            
            Serial.println("[SYSTEM] System updated successfully. Ready for operation.");
        } else {
            Serial.println("[ERROR] Failed to load the newly saved configuration into the Analyzer.");
        }
    } else {
        Serial.println("[ERROR] Failed to open config.json for writing.");
    }
}

void setup() {
    Serial.begin(115200);
    // while (!Serial) { delay(10); } // ESPERA DE SERIAL ELIMINADA para permitir uso con fuente de alimentación externa
    delay(1000); // Allow serial monitor to catch up
    Serial.println("\n=========================================");
    Serial.println("   INDUSTRIAL PREDICTIVE MAINT SYSTEM    ");
    Serial.println("=========================================");

    // --- Configuración e Inicialización de Hardware (Buzzer y MPU6050) ---
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Asegurarse de que el buzzer esté apagado

    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
    if (!mpu.begin(0x68, &Wire, 0)) {
        Serial.println("[ERROR] No se pudo encontrar el MPU6050. Revisa el cableado.");
    } else {
        Serial.println("[SYSTEM] MPU6050 inicializado correctamente.");
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    
    // --- 1. Initialize Non-Volatile File System (LittleFS) ---
    if (!LittleFS.begin(true)) {
        Serial.println("[CRITICAL ERROR] LittleFS mount failed. Halting system.");
        while (true) { delay(1000); } // Halt the processor if storage fails
    }

    // --- 2. Load Configuration or Enter Factory Mode ---
    // By default, we pass an empty string. The NetworkManager knows that an empty 
    // string means "I am a new device, generate my ID based on my MAC address".
    String currentDeviceId = ""; 
    
    if (LittleFS.exists(CONFIG_FILE_PATH)) {
        Serial.println("[SYSTEM] Found existing config.json. Attempting to load...");
        
        if (analyzer.loadConfig(CONFIG_FILE_PATH)) {
            // Success! The device has been configured before. We grab its assigned name.
            currentDeviceId = analyzer.getConfig().machineName;
            Serial.println("[SYSTEM] Loaded config for machine: " + currentDeviceId);
        } else {
            Serial.println("[WARNING] config.json exists but parsing failed. Booting in Factory Mode.");
        }
    } else {
        Serial.println("[SYSTEM] No config.json found. Booting in Factory/Setup Mode.");
    }

    // --- 3. Initialize Network and MQTT ---
    // If currentDeviceId is "", it will auto-name itself "AIoT-<MAC>" and 
    // subscribe to "maquinas/APC-<MAC>/config".
    // If it has a saved name (e.g., "Motor_Pomba_1"), it will use that.
    network.begin(currentDeviceId);
    
    // Attach the callback function to handle incoming setup commands
    network.setConfigCallback(onConfigurationReceived);
}

void loop() {
    // --- 1. Network Maintenance (Asynchronous) ---
    // This handles Wi-Fi reconnections, MQTT keep-alive pings, and processes 
    // incoming JSON payloads without blocking the processor.
    network.loop(); 

    // --- 2. Sensor Data Acquisition & Edge AI Processing ---
    // Using a non-blocking timer (millis) instead of delay() ensures the network
    // stack is never starved of CPU time.
    static unsigned long lastAnalysisTime = 0;
    unsigned long interval = analyzer.getConfig().isLoaded ? 
    analyzer.getConfig().samplingRate : 1000;
    
    if (millis() - lastAnalysisTime >= interval) { 
        lastAnalysisTime = millis();
        
        // Lectura del sensor MPU6050 en lugar del dummy buffer
        float realBuffer[100]; 
        for(int i = 0; i < 100; i++) {
            sensors_event_t a, g, temp;
            
            // Inicializar las estructuras a 0 para borrar "basura de memoria"
            memset(&a, 0, sizeof(a));
            memset(&g, 0, sizeof(g));
            memset(&temp, 0, sizeof(temp));

            // Si se lee correctamente, usamos la magnitud real. Si falla, queda en 0.
            if (mpu.getEvent(&a, &g, &temp)) {
                // Usamos la magnitud de la aceleración para el análisis de vibración
                realBuffer[i] = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2)); 
            } else {
                realBuffer[i] = 0.0f; 
            }
            delay(2); // Retardo pequeño para espaciar las muestras
        }

        // Execute Edge processing: ISO 20816 Math (RMS, Variance, FFT, etc.)
        AnalysisResult result = analyzer.analyze(realBuffer, 100);
        String zoneLabel = analyzer.getZoneString(result.zone); // e.g., "ZONE_A_GOOD", "ZONE_D_DANGER"

        // Publish the telemetry data to the MQTT broker
        network.publishTelemetry(result.rms, result.variance, zoneLabel);

        // Activar el buzzer (pin 11) cuando supera los límites de la zona C de la norma ISO
        // (es decir, cuando rebasa el umbral inaceptable y entra en Zona D)
        if (result.zone == IsoZone::ZONE_B) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
            delay(500);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
            delay(500);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
            delay(500);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
        } else if (result.zone == IsoZone::ZONE_C)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(1000);
            digitalWrite(BUZZER_PIN, LOW);
            delay(500);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(1000);
            digitalWrite(BUZZER_PIN, LOW);
            delay(500);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(1000);
            digitalWrite(BUZZER_PIN, LOW);
            delay(500);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(1000);
            digitalWrite(BUZZER_PIN, LOW);
        }
        else if (result.zone == IsoZone::ZONE_D)
        {
            digitalWrite(BUZZER_PIN, HIGH);
        }
        
        else {
            digitalWrite(BUZZER_PIN, LOW);
        }
    }
}