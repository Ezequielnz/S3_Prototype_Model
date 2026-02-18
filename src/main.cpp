#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Configuración de pines según la tabla anterior
const int MIC_PIN = 1;     // ADC para el MAX9814
const int TEMP_RX_PIN = 4; // RX para el módulo de temperatura

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, TEMP_RX_PIN, -1); // Puerto para el sensor de temperatura

  // Iniciar I2C en los pines GP8 y GP9 del Super Mini
  Wire.begin(8, 9); 

  if (!mpu.begin()) {
    Serial.println("Error: No se encontró el MPU6050");
    while (1) yield();
  }
  Serial.println("MPU6050 Listo!");
}

void loop() {
  // 1. Probar MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Acel X: "); Serial.print(a.acceleration.x);

  // 2. Probar Micrófono (MAX9814)
  int micValue = analogRead(MIC_PIN);
  Serial.print(" | Mic: "); Serial.print(micValue);

  // 3. Probar Temperatura (Lectura Serial del módulo Starware)
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    Serial.print(" | Temp Raw: "); Serial.print(data);
  }

  Serial.println("");
  delay(500);
}