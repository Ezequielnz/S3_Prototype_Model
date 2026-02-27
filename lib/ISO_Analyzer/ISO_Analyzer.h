// Declare the vibration code structure: function names, classes, and variables other files can use.
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <vector>

// Enumeration for the 4 zones of the ISO 20816 standard
enum class IsoZone {
    ZONE_A, // Satisfactory (New commissioning)
    ZONE_B, // Long-term operation without restrictions
    ZONE_C, // Limited operation (Alert)
    ZONE_D, // Risk of damage (Danger)
    UNKNOWN
};

// Structure for the current machine thresholds
struct IsoThresholds {
    float satisfactory;
    float unsatisfactory;
    float unacceptable;
};

// Structure to store the complete configuration
struct MachineConfig {
    String machineName;
    String isoGroup;
    String support;
    IsoThresholds thresholds;
    int samplingRate;
    bool isLoaded = false;
};

// Structure to return analysis results
struct AnalysisResult {
    float rms;
    float variance;
    IsoZone zone;
};

class ISO_Analyzer {
private:
    MachineConfig currentConfig;

    // Internal mathematical calculations
    float calculateMean(const float* data, size_t length);

public:
    ISO_Analyzer();

    // Load configuration from LittleFS
    bool loadConfig(const char* filepath);

    // Run mathematical and zone analysis
    AnalysisResult analyze(const float* rawData, size_t length);

    // Get current configuration (useful for debugging)
    MachineConfig getConfig();
    
    // Convert the zone enum to a String for sending via WiFi/MQTT
    String getZoneString(IsoZone zone);
};