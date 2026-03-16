#include "NetworkManager.h"
#include "config.h"

NetworkManager* NetworkManager::_instance = nullptr;

NetworkManager::NetworkManager(String ssid, String password, String mqttServer, int port) {
    _ssid = ssid;
    _password = password;
    _mqttServer = mqttServer;
    _mqttPort = port;
    _instance = this;
}

String NetworkManager::getCleanMacAddress() {
    String mac = WiFi.macAddress();
    mac.replace(":", ""); // Remove colons for a cleaner ID
    return mac;
}

void NetworkManager::generateDeviceIdentity() {
    String mac = getCleanMacAddress();
    
    // Cumpliendo con tus reglas estrictas de hardware nuevo
    _deviceId = "AIoT-" + mac;
    _mqttConfigTopic = "maquinas/APC-" + mac + "/config";
    
    Serial.println("\n--- Device Identity Configured ---");
    Serial.println("Device Name: " + _deviceId);
    Serial.println("Config Topic: " + _mqttConfigTopic);
}

void NetworkManager::begin(String deviceId) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(_ssid);

    WiFi.begin(_ssid.c_str(), _password.c_str());
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected.");
    
    // Lógica de identidad: Si no se le pasa un ID (dispositivo nuevo de fábrica), auto-generamos
    if (deviceId == "" || deviceId == FALLBACK_DEVICE_ID) {
        generateDeviceIdentity();
    } else {
        _deviceId = deviceId;
        // Si más adelante pasas la config vía LoRa, reconstruimos el tópico en base al nuevo nombre
        _mqttConfigTopic = "maquinas/APC-" + _deviceId + "/config"; 
    }
    
    _mqttClient.setClient(_espClient);
    _mqttClient.setServer(_mqttServer.c_str(), _mqttPort);
    _mqttClient.setCallback(NetworkManager::mqttCallback);
}

void NetworkManager::updateDeviceId(String newDeviceId) {
    if (_deviceId != newDeviceId) {
        _deviceId = newDeviceId;
        Serial.println("Device ID updated. Reconnecting MQTT to apply new topics...");
        _mqttClient.disconnect(); // Forces reconnection with new topics in the next loop
    }
}

void NetworkManager::reconnectMQTT() {
    while (!_mqttClient.connected()) {
        Serial.print("Attempting MQTT connection as ");
        Serial.print(_deviceId);
        Serial.print("...");
        
        // El dispositivo se conecta al Broker asumiendo su propio nombre (AIoT-MAC)
        if (_mqttClient.connect(_deviceId.c_str())) {
            Serial.println(" Connected to Broker.");
            
            // Se suscribe al tópico correcto generado (maquinas/APC-MAC/config)
            _mqttClient.subscribe(_mqttConfigTopic.c_str());
            Serial.println("Subscribed to: " + _mqttConfigTopic);
        } else {
            Serial.print(" Failed, rc=");
            Serial.print(_mqttClient.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000); 
        }
    }
}

void NetworkManager::mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (_instance) {
        _instance->processIncomingMessage(topic, payload, length);
    }
}

void NetworkManager::processIncomingMessage(char* topic, byte* payload, unsigned int length) {
    Serial.println("\nNew MQTT Message received.");

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
        Serial.print("JSON Parse Error: ");
        Serial.println(error.c_str());
        return;
    }

    if (_onConfigReceived) {
        _onConfigReceived(doc);
    }
}

void NetworkManager::loop() {
    if (!_mqttClient.connected()) {
        reconnectMQTT();
    }
    _mqttClient.loop();
}

void NetworkManager::setConfigCallback(ConfigCallback callback) {
    _onConfigReceived = callback;
}

void NetworkManager::publishTelemetry(float rms, float variance, String zone) {
    if (!_mqttClient.connected()) return;

    JsonDocument doc;
    doc["device"] = _deviceId;
    doc["rms"] = rms;
    doc["variance"] = variance;
    doc["zone"] = zone;
    
    char buffer[256];
    serializeJson(doc, buffer);

    // Actualicé este tópico para que mantenga coherencia con español, aunque es tu función original
    String telemetryTopic = "maquinas/APC-" + getCleanMacAddress() + "/telemetry";
    _mqttClient.publish(telemetryTopic.c_str(), buffer);
}