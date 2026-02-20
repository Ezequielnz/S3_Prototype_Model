#include "NetworkManager.h"

NetworkManager* NetworkManager::_instance = nullptr;

NetworkManager::NetworkManager(String ssid, String password, String mqttServer, int port) {
    _ssid = ssid;
    _password = password;
    _mqttServer = mqttServer;
    _mqttPort = port;
    _instance = this;
}

void NetworkManager::begin(String deviceId) {
    _deviceId = deviceId;
    
    Serial.print("Connecting to WiFi: ");
    Serial.println(_ssid);

    WiFi.begin(_ssid.c_str(), _password.c_str());
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected.");
    
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
        Serial.print("Attempting MQTT connection...");
        
        // Generate a unique client ID for the broker connection
        String clientId = "ESP32-" + _deviceId + "-" + String(random(0xffff), HEX);
        
        if (_mqttClient.connect(clientId.c_str())) {
            Serial.println("Connected to Broker.");
            
            // SUBSCRIBE: Listen for configurations for THIS specific machine
            String topic = "machines/" + _deviceId + "/config";
            _mqttClient.subscribe(topic.c_str());
            Serial.println("Subscribed to: " + topic);
        } else {
            Serial.print("Failed, rc=");
            Serial.print(_mqttClient.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000); // Note: In a real non-blocking scenario, use millis() here instead of delay()
        }
    }
}

void NetworkManager::mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (_instance) {
        _instance->processIncomingMessage(topic, payload, length);
    }
}

void NetworkManager::processIncomingMessage(char* topic, byte* payload, unsigned int length) {
    Serial.println("New MQTT Message received.");

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
    _mqttClient.loop(); // Keeps the MQTT connection alive and checks for incoming messages
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

    String topic = "machines/" + _deviceId + "/telemetry";
    _mqttClient.publish(topic.c_str(), buffer);
}