#pragma once

#ifndef CONFIG_H
#define CONFIG_H
// Network Credentials
#define WIFI_SSID "iPhone de Ezequiel"
#define WIFI_PASS "Exequielnz1"
#define MQTT_BROKER_IP "172.20.10.5"
#define MQTT_PORT 1883

// File System
#define CONFIG_FILE_PATH "/config.json"

// Fallback ID if the device is factory-new and has no JSON
#define FALLBACK_DEVICE_ID "UNCONFIGURED_NODE" 

// Hardware Pins
#define MPU_SDA_PIN 8
#define MPU_SCL_PIN 9
#define MPU_INT_PIN 10
#define BUZZER_PIN 11

// Data Acquisition
#define SAMPLE_SIZE 1024 // Adjust based on your sensor buffer

#endif // CONFIG_H
