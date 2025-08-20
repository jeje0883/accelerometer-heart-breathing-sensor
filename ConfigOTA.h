#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>


struct AlgoConfig {
float sampleHz = 50.0f;
float breathCutoffHz = 0.5f;
float hpMinDeltaG = 0.005f;
float breathThreshG = 0.02f;
uint32_t breathRefractMs = 1500;
uint32_t beatRefractMs = 400;
uint32_t reportWindowMs = 60000;
uint32_t calibrateMs = 3000;
};


class ConfigOTA {
public:
// Singleton style (or create your own instance)
static ConfigOTA& instance();


// Initialize FS, load config, and (optionally) bring up Wi‑Fi + server
bool begin(
const char* ssid,
const char* pass,
bool enableAPFallback = true,
const char* apSsid = "ESP32-Config",
const char* apPass = "12345678",
uint32_t wifiTimeoutMs = 10000
);


// Call regularly in loop()
void handle();


// Access current config
const AlgoConfig& get() const { return cfg_; }
AlgoConfig& mutableConfig() { return cfg_; }


// Save current cfg_ to /config.json
bool save();


// Force reload from disk (overwrites cfg_)
bool load();


// HTTP server status
bool serverRunning() const { return serverRunning_; }


// Utilities
String ipAddress() const;


private:
ConfigOTA() = default;
ConfigOTA(const ConfigOTA&) = delete;
ConfigOTA& operator=(const ConfigOTA&) = delete;


bool mountFS_();
void setupServer_();
void registerRoutes_();
void respondJson_(int code, const String& body);


bool connectSTA_(const char* ssid, const char* pass, uint32_t timeoutMs);
bool startAP_(const char* ssid, const char* pass);


bool decodeAndApplyJson_(Stream& s);
bool loadFromFile_(const char* path);


private:
WebServer server_{80};
bool fsMounted_ = false;
bool serverRunning_ = false;
AlgoConfig cfg_{};
};
