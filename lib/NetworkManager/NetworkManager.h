#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Type definition for the configuration callback function
typedef void (*ConfigCallback)(JsonDocument& doc);

class NetworkManager {
private:
    String _ssid;
    String _password;
    String _mqttServer;
    int _mqttPort;
    String _deviceId; // Now dynamic, not a constant char*
    
    WiFiClient _espClient;
    PubSubClient _mqttClient;
    ConfigCallback _onConfigReceived; 

    void reconnectMQTT();
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
    static NetworkManager* _instance; 

public:
    NetworkManager(String ssid, String password, String mqttServer, int port);
    
    void begin(String deviceId);
    void loop();
    
    // Allows updating the MQTT topic dynamically if the JSON changes the machine name
    void updateDeviceId(String newDeviceId); 
    
    void setConfigCallback(ConfigCallback callback);
    void publishTelemetry(float rms, float variance, String zone);
    void processIncomingMessage(char* topic, byte* payload, unsigned int length);
};