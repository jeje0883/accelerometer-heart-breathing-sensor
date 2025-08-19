#include <Wire.h>
#include "MPU6050.h"   // Jeff Rowberg library

// ==== I2C pins for ESP32 ====
#define I2C_SDA 21
#define I2C_SCL 22

MPU6050 mpu;

// ---- Tuning constants ----
const float SAMPLE_HZ           = 50.0;          // ~50 Hz sampling
const uint32_t SAMPLE_DELAY_MS  = 1000.0 / SAMPLE_HZ;

const float BREATH_CUTOFF_HZ    = 0.5;           // low-pass ~0.5 Hz
const float HP_MIN_DELTA_G      = 0.005;         // heartbeat motion threshold (tune)
const float BREATH_THRESH_G     = 0.02;          // breath peak threshold (tune)

const uint32_t BREATH_REFRACT_MS = 1500;         // min 1.5s between breaths (<= 40 bpm)
const uint32_t BEAT_REFRACT_MS   = 400;          // min 0.4s between beats (<= 150 bpm)
const uint32_t REPORT_WINDOW_MS  = 60000;        // report every 60s
const uint32_t CALIBRATE_MS      = 3000;         // 3s baseline calibration

// ---- State ----
float accelZFiltered = 0.0f;     // low-pass (breathing)
float accelZBaseline = 0.0f;     // Z baseline (gravity/orientation)
unsigned long lastBreathTime = 0, lastBeatTime = 0;
int breathCount = 0, beatCount = 0;
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Faster I2C improves read latency.
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz

  mpu.initialize();

  // Optional: configure ranges & digital low-pass to reduce noise
  // (Values depend on your library version; these are common calls.)
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // finest resolution
  mpu.setDLPFMode(MPU6050_DLPF_BW_10);             // ~10 Hz bandwidth

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1) { delay(1000); }
  }
  Serial.println("MPU6050 ready. Keep still for baseline calibration...");

  // ---- Baseline calibration: average Z for a few seconds ----
  unsigned long t0 = millis();
  long sum = 0;
  long n = 0;
  while (millis() - t0 < CALIBRATE_MS) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    sum += az;
    n++;
    delay(5);
  }
  if (n > 0) {
    float az_avg_counts = (float)sum / (float)n;  // raw counts
    accelZBaseline = az_avg_counts / 16384.0f;    // convert to g
  }
  accelZFiltered = 0.0f; // start LPF around 0 after baseline removal

  Serial.print("Baseline Z (g): ");
  Serial.println(accelZBaseline, 4);

  startTime = millis();
}

void loop() {
  // --- Read accel ---
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert to g's and remove baseline/gravity/orientation offset
  float accelZ = (az / 16384.0f) - accelZBaseline;

  // --- Low-pass filter for breathing ---
  // One-pole LPF with cutoff ~ BREATH_CUTOFF_HZ
  // alpha = dt / (RC + dt), RC = 1/(2*pi*fc)
  const float dt   = 1.0f / SAMPLE_HZ;
  const float RC   = 1.0f / (2.0f * 3.1415926f * BREATH_CUTOFF_HZ);
  const float alpha = dt / (RC + dt);
  accelZFiltered += alpha * (accelZ - accelZFiltered);

  // --- Heartbeat proxy: high-pass-ish component ---
  float highpass = accelZ - accelZFiltered;   // remove slow breathing component

  unsigned long now = millis();

  // ---- Breath detection (peak crossing + refractory) ----
  if (accelZFiltered > BREATH_THRESH_G && (now - lastBreathTime) > BREATH_REFRACT_MS) {
    breathCount++;
    lastBreathTime = now;
  }

  // ---- Heartbeat detection (tiny rapid changes + refractory) ----
  if (fabs(highpass) > HP_MIN_DELTA_G && (now - lastBeatTime) > BEAT_REFRACT_MS) {
    beatCount++;
    lastBeatTime = now;
  }

  // ---- Periodic report ----
  if (now - startTime >= REPORT_WINDOW_MS) {
    Serial.print("Breaths per minute: ");
    Serial.println(breathCount);
    Serial.print("Heartbeats per minute: ");
    Serial.println(beatCount);

    breathCount = 0;
    beatCount   = 0;
    startTime   = now;
  }

  delay(SAMPLE_DELAY_MS); // ~50 Hz
}
