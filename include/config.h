#pragma once

#ifndef CONFIG_H
#define CONFIG_H
// Network Credentials
#define WIFI_SSID "WIFI-NUÑEZ-ALARCON"
#define WIFI_PASS "namuebles2025"
#define MQTT_BROKER_IP "192.168.1.110"
#define MQTT_PORT 1883

// File System
#define CONFIG_FILE_PATH "/config.json"

// Fallback ID if the device is factory-new and has no JSON
#define FALLBACK_DEVICE_ID "UNCONFIGURED_NODE" 

// Data Acquisition
#define SAMPLE_SIZE 1024 // Adjust based on your sensor buffer

#endif // CONFIG_H
