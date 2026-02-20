#include <Arduino.h>
#include <LittleFS.h>
#include "config.h"
#include "ISO_Analyzer.h"
#include "NetworkManager.h"

ISO_Analyzer analyzer;
NetworkManager network(WIFI_SSID, WIFI_PASS, MQTT_BROKER_IP, MQTT_PORT);

// Callback executed when Python sends a new JSON via MQTT
void onConfigurationReceived(JsonDocument& doc) {
    Serial.println("Applying new configuration from PC...");
    
    File file = LittleFS.open(CONFIG_FILE_PATH, "w");
    if (file) {
        serializeJson(doc, file);
        file.close();
        
        // Reload analyzer with new data
        analyzer.loadConfig(CONFIG_FILE_PATH);
        
        // Update network manager in case the machine name changed
        String newMachineName = analyzer.getConfig().machineName;
        network.updateDeviceId(newMachineName);
        
        Serial.println("System updated successfully.");
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize File System
    if (!LittleFS.begin(true)) {
        Serial.println("Critical Error: LittleFS mount failed");
        return;
    }

    // 1. Try to load existing configuration
    String currentDeviceId = FALLBACK_DEVICE_ID;
    if (analyzer.loadConfig(CONFIG_FILE_PATH)) {
        currentDeviceId = analyzer.getConfig().machineName;
        Serial.println("Loaded config for: " + currentDeviceId);
    } else {
        Serial.println("No config found. Using fallback ID.");
    }

    // 2. Initialize Network with the loaded (or fallback) ID
    network.begin(currentDeviceId);
    network.setConfigCallback(onConfigurationReceived);
}

void loop() {
    network.loop(); // Handles Wi-Fi/MQTT asynchronously

    // Sensor acquisition loop (Non-blocking)
    static unsigned long lastAnalysisTime = 0;
    if (millis() - lastAnalysisTime > 1000) { 
        lastAnalysisTime = millis();
        
        // Simulated dummy data (Replace with I2S/ADC sensor reading)
        float dummyBuffer[100]; 
        for(int i=0; i<100; i++) dummyBuffer[i] = random(0, 50) / 10.0; 

        // Execute ISO 20816 Math
        AnalysisResult result = analyzer.analyze(dummyBuffer, 100);
        String zoneLabel = analyzer.getZoneString(result.zone);

        // Send results to Python/Broker
        network.publishTelemetry(result.rms, result.variance, zoneLabel);
    }
}