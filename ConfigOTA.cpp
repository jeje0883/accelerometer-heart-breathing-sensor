#include "ConfigOTA.h"
if (!save()) {
respondJson_(500, "{\"error\":\"save failed\"}");
return;
}
respondJson_(200, "{\"ok\":true}");
});


server_.on("/reboot", HTTP_GET, [this](){
server_.send(200, "text/plain", "Rebooting...");
delay(500);
ESP.restart();
});
}


void ConfigOTA::respondJson_(int code, const String& body) {
server_.send(code, "application/json", body);
}


bool ConfigOTA::connectSTA_(const char* ssid, const char* pass, uint32_t timeoutMs) {
if (!ssid || !*ssid) return false;
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, pass);
Serial.printf("[ConfigOTA] Connecting to %s...\n", ssid);
uint32_t t0 = millis();
while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
delay(200);
Serial.print(".");
}
Serial.println();
if (WiFi.status() == WL_CONNECTED) {
Serial.printf("[ConfigOTA] STA IP: %s\n", WiFi.localIP().toString().c_str());
return true;
}
Serial.println("[ConfigOTA] STA connect failed");
return false;
}


bool ConfigOTA::startAP_(const char* ssid, const char* pass) {
WiFi.mode(WIFI_AP);
bool ok = WiFi.softAP(ssid, pass);
if (ok) {
Serial.printf("[ConfigOTA] AP IP: %s (SSID:%s)\n", WiFi.softAPIP().toString().c_str(), ssid);
} else {
Serial.println("[ConfigOTA] AP start failed");
}
return ok;
}


bool ConfigOTA::decodeAndApplyJson_(Stream& s) {
StaticJsonDocument<512> doc;
DeserializationError err = deserializeJson(doc, s);
if (err) return false;


if (doc.containsKey("sampleHz")) cfg_.sampleHz = doc["sampleHz"].as<float>();
if (doc.containsKey("breathCutoffHz")) cfg_.breathCutoffHz = doc["breathCutoffHz"].as<float>();
if (doc.containsKey("hpMinDeltaG")) cfg_.hpMinDeltaG = doc["hpMinDeltaG"].as<float>();
if (doc.containsKey("breathThreshG")) cfg_.breathThreshG = doc["breathThreshG"].as<float>();
if (doc.containsKey("breathRefractMs")) cfg_.breathRefractMs= doc["breathRefractMs"].as<uint32_t>();
if (doc.containsKey("beatRefractMs")) cfg_.beatRefractMs = doc["beatRefractMs"].as<uint32_t>();
if (doc.containsKey("reportWindowMs")) cfg_.reportWindowMs = doc["reportWindowMs"].as<uint32_t>();
if (doc.containsKey("calibrateMs")) cfg_.calibrateMs = doc["calibrateMs"].as<uint32_t>();
return true;
}


bool ConfigOTA::loadFromFile_(const char* path) {
if (!LittleFS.exists(path)) {
Serial.println("[ConfigOTA] No config file, writing defaults");
return save();
}
File f = LittleFS.open(path, "r");
if (!f) return false;
bool ok = decodeAndApplyJson_(f);
f.close();
if (ok) Serial.println("[ConfigOTA] Config loaded");
return ok;
}
