#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

// Variables for breathing & heartbeat
float accelZFiltered = 0;
unsigned long lastBreathTime = 0, lastBeatTime = 0;
int breathCount = 0, beatCount = 0;
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  
  Serial.println("MPU6050 ready!");
  startTime = millis();
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert to g's
  float accelZ = az / 16384.0;

  // Low-pass filter for breathing (cutoff ~0.5 Hz)
  accelZFiltered = 0.9 * accelZFiltered + 0.1 * accelZ;

  // Detect breathing peaks (slow variations)
  if (accelZFiltered > 0.02 && millis() - lastBreathTime > 2000) { 
    breathCount++;
    lastBreathTime = millis();
  }

  // Detect heartbeat peaks (small rapid changes, band-pass idea)
  if (abs(accelZ - accelZFiltered) > 0.005 && millis() - lastBeatTime > 400) {
    beatCount++;
    lastBeatTime = millis();
  }

  // Every 60 seconds, print rates
  if (millis() - startTime >= 60000) {
    Serial.print("Breaths per minute: "); Serial.println(breathCount);
    Serial.print("Heartbeats per minute: "); Serial.println(beatCount);

    breathCount = 0;
    beatCount = 0;
    startTime = millis();
  }

  delay(20); // ~50 Hz sampling
}
