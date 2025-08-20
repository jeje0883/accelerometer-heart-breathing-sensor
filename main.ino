#include <Wire.h>
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


// Heartbeat proxy component
float highpass = accelZ - accelZFiltered;


unsigned long now = millis();


// Breath detection
if (accelZFiltered > cfg.breathThreshG && (now - lastBreathTime) > cfg.breathRefractMs) {
breathCount++;
lastBreathTime = now;
}


// Heartbeat detection
if (fabs(highpass) > cfg.hpMinDeltaG && (now - lastBeatTime) > cfg.beatRefractMs) {
beatCount++;
lastBeatTime = now;
}


// Periodic report
if (now - startTime >= cfg.reportWindowMs) {
Serial.print("Breaths per minute: "); Serial.println(breathCount);
Serial.print("Heartbeats per minute: "); Serial.println(beatCount);
breathCount = 0; beatCount = 0; startTime = now;
}


delay(sampleDelayMs);
}
