#include <Wire.h>
#include "MPU6050.h" // Jeff Rowberg library
#include "ConfigOTA.h"


// ==== I2C pins for ESP32 ====
#define I2C_SDA 21
#define I2C_SCL 22


MPU6050 mpu;


// ---- State ----
float accelZFiltered = 0.0f; // low-pass (breathing)
float accelZBaseline = 0.0f; // Z baseline (gravity/orientation)
unsigned long lastBreathTime = 0, lastBeatTime = 0;
int breathCount = 0, beatCount = 0;
unsigned long startTime;


static inline float lpfStep(float current, float target, float alpha){
return current + alpha * (target - current);
}


void setup() {
Serial.begin(115200);
delay(200);


// 1) Start Config OTA (connect STA, else AP fallback); change creds as needed
ConfigOTA::instance().begin(
"YourWiFiSSID", "YourWiFiPASS",
/*enableAPFallback=*/true,
/*apSsid=*/"ESP32-Config",
/*apPass=*/"12345678",
/*wifiTimeoutMs=*/8000
);


// 2) I2C
Wire.begin(I2C_SDA, I2C_SCL);
Wire.setClock(400000);


// 3) Sensor init
mpu.initialize();
mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
mpu.setDLPFMode(MPU6050_DLPF_BW_10);
if (!mpu.testConnection()) {
Serial.println("MPU6050 connection failed!");
while (1) { delay(1000); }
}
Serial.println("MPU6050 ready. Keep still for baseline calibration...");


// 4) Baseline calibration using cfg.calibrateMs
const AlgoConfig& cfg = ConfigOTA::instance().get();
unsigned long t0 = millis();
long sum = 0; long n = 0;
while (millis() - t0 < cfg.calibrateMs) {
int16_t ax, ay, az; mpu.getAcceleration(&ax, &ay, &az);
sum += az; n++; delay(5);
}
if (n > 0) {
float az_avg_counts = (float)sum / (float)n; // raw counts
accelZBaseline = az_avg_counts / 16384.0f; // convert to g
}
accelZFiltered = 0.0f;


Serial.print("Baseline Z (g): ");
Serial.println(accelZBaseline, 4);


startTime = millis();
}


void loop() {
ConfigOTA::instance().handle(); // non-blocking HTTP portal


// Read accel
int16_t ax, ay, az;
mpu.getAcceleration(&ax, &ay, &az);


const AlgoConfig& cfg = ConfigOTA::instance().get();


// dt from sampleHz
const float dt = 1.0f / cfg.sampleHz;
const uint32_t sampleDelayMs = (uint32_t)(1000.0f / cfg.sampleHz);


// Convert to g's and remove baseline
float accelZ = (az / 16384.0f) - accelZBaseline;


// LPF with cutoff ~ cfg.breathCutoffHz
const float RC = 1.0f / (2.0f * 3.1415926f * cfg.breathCutoffHz);
const float alpha = dt / (RC + dt);
accelZFiltered = lpfStep(accelZFiltered, accelZ, alpha);


}
