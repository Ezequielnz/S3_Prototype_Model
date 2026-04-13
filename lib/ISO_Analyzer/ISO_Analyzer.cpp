#include "ISO_Analyzer.h"
// implement the actual function code from ISO_Analyzer.h

ISO_Analyzer::ISO_Analyzer() {
    // Default initialization
}

bool ISO_Analyzer::loadConfig(const char* filepath) {
    if (!LittleFS.begin()) {
        Serial.println("Error: No se pudo montar LittleFS");
        return false;
    }

    File configFile = LittleFS.open(filepath, "r");
    if (!configFile) {
        Serial.println("Error: No se pudo abrir el archivo de configuración");
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();

    if (error) {
        Serial.print("Error al parsear el JSON: ");
        Serial.println(error.c_str());
        return false;
    }

    // Assign JSON values to our structure
    currentConfig.machineName = doc["machine_name"].as<String>();
    currentConfig.isoGroup = doc["iso_group"].as<String>();
    currentConfig.support = doc["support"].as<String>();
    
    JsonObject thresholds = doc["thresholds"];
    currentConfig.thresholds.satisfactory = thresholds["satisfactory"];
    currentConfig.thresholds.unsatisfactory = thresholds["unsatisfactory"];
    currentConfig.thresholds.unacceptable = thresholds["unacceptable"];
    
    currentConfig.samplingRate = doc["sampling_rate"];
    currentConfig.isLoaded = true;

    return true;
}

float ISO_Analyzer::calculateMean(const float* data, size_t length) {
    if (length == 0) return 0.0f;
    float sum = 0.0f;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum / length;
}

AnalysisResult ISO_Analyzer::analyze(const float* rawData, size_t length) {
    AnalysisResult result = {0.0f, 0.0f, IsoZone::UNKNOWN};
    if (length == 0) return result;

    float mean = calculateMean(rawData, length);
    
    // Definir intervalo de tiempo dt según la frecuencia de muestreo de la configuración.
    // Si no es válido, aproxima a 4ms (250Hz) para compensar el retardo de I2C.
    float dt = (currentConfig.samplingRate > 0) ? (1.0f / currentConfig.samplingRate) : 0.004f;
    
    std::vector<float> velocity(length, 0.0f);
    float current_velocity = 0.0f;
    
    // 1. Convertir aceleración a velocidad mediante integración numérica
    // Constante de Filtro Pasa Altos (Leaky Integrator) para prevenir el 'drift' (desvío al infinito)
    const float alpha = 0.98f; 
    
    for (size_t i = 0; i < length; i++) {
        // Remover gravedad / componente DC de la aceleración
        float accel_without_gravity = rawData[i] - mean; 
        
        // Integrar en el tiempo: v = a * dt. Multiplicar por 1000 para pasar de m/s a mm/s
        // Se aplica 'alpha' (Leaky Integrator) para atenuar las bajas frecuencias.
        current_velocity = alpha * current_velocity + (accel_without_gravity * dt * 1000.0f);
        velocity[i] = current_velocity;
    }

    // 2. Calcular el RMS de la Velocidad en mm/s resultante
    float vel_mean = calculateMean(velocity.data(), length);
    float sumSquaresVar = 0.0f;
    
    for (size_t i = 0; i < length; i++) {
        float diff = velocity[i] - vel_mean; // Eliminar offset residual de la velocidad
        sumSquaresVar += diff * diff;
    }

    // El resultado final que evalúa la norma ISO 20816 es el RMS de la Velocidad, en mm/s
    result.rms = sqrt(sumSquaresVar / length);
    result.variance = sumSquaresVar / length;

    // Classification logic according to the ISO 20816 standard and dynamic JSON
    if (!currentConfig.isLoaded) {
        result.zone = IsoZone::UNKNOWN;
    } else if (result.rms <= currentConfig.thresholds.satisfactory) {
        result.zone = IsoZone::ZONE_A;
    } else if (result.rms <= currentConfig.thresholds.unsatisfactory) {
        result.zone = IsoZone::ZONE_B;
    } else if (result.rms <= currentConfig.thresholds.unacceptable) {
        result.zone = IsoZone::ZONE_C;
    } else {
        result.zone = IsoZone::ZONE_D;
    }

    return result;
}

MachineConfig ISO_Analyzer::getConfig() {
    return currentConfig;
}

String ISO_Analyzer::getZoneString(IsoZone zone) {
    switch(zone) {
        case IsoZone::ZONE_A: return "Zona A (Satisfactorio)";
        case IsoZone::ZONE_B: return "Zona B (Operación a largo plazo)";
        case IsoZone::ZONE_C: return "Zona C (Operación limitada)";
        case IsoZone::ZONE_D: return "Zona D (Riesgo de daño)";
        default: return "Desconocido";
    }
}