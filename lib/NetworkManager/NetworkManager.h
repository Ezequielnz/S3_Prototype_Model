#pragma once

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

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
    
    String _deviceId;        // Se llamará: AIoT-(MAC)
    String _mqttConfigTopic; // Será: maquinas/APC-(MAC)/config
    
    WiFiClient _espClient;
    PubSubClient _mqttClient;
    ConfigCallback _onConfigReceived; 

    void reconnectMQTT();
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
    static NetworkManager* _instance; 

    // Declaraciones faltantes agregadas aquí
    String getCleanMacAddress();
    void generateDeviceIdentity();

public:
    NetworkManager(String ssid, String password, String mqttServer, int port);
    
    // begin() ahora usa un valor por defecto vacío para auto-generar la MAC si es nuevo
    void begin(String deviceId = ""); 
    void loop();
    
    void updateDeviceId(String newDeviceId); 
    
    void setConfigCallback(ConfigCallback callback);
    void publishTelemetry(float rms, float variance, String zone);
    void processIncomingMessage(char* topic, byte* payload, unsigned int length);
};

#endif // NETWORK_MANAGER_H