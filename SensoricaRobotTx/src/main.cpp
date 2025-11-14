#if defined(ESP32)
#include <WiFi.h>
#include <TinyGPSPlus.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <HTTPClient.h>
#else
#error "This sketch requires an ESP8266 or ESP32 board."
#endif

#include <Wire.h>

#if defined(ESP32)
constexpr uint8_t HDC1080_SDA_PIN = 21;
constexpr uint8_t HDC1080_SCL_PIN = 22;
#endif
// Texas Instruments HDC1080 humidity/temp sensor (I2C)
constexpr uint8_t HDC1080_ADDR = 0x40;
constexpr unsigned long ENV_SAMPLE_PERIOD_MS = 2000;
constexpr unsigned long HDC1080_MEAS_DELAY_MS = 200;

// Wi-Fi and radar broker configuration
const char* WIFI_SSID = "UPBWiFi";
const char* WIFI_PASSWORD = "";
// Control broker public endpoint (set to your deployment host)
WiFiClient client;
const char* SERVER_URL = "3.208.15.83";

constexpr unsigned long WIFI_RETRY_MS = 5000;

unsigned long lastWifiAttempt = 0;

bool envSensorInitialized = false;
bool envSensorHasSample = false;
float envLastTemperatureC = 0.0f;
float envLastHumidityPct = 0.0f;
unsigned long envLastSampleMs = 0;

#if defined(ESP32)
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;
constexpr int GPS_RX_PIN = 34;
constexpr int GPS_TX_PIN = 12;
constexpr uint32_t GPS_BAUD_RATE = 9600;
constexpr unsigned long GPS_MAX_AGE_MS = 5000;
double gpsLastLat = 0.0;
double gpsLastLon = 0.0;
double gpsLastAlt = 0.0;
double gpsLastSpeedMps = 0.0;
double gpsLastHdop = 0.0;
uint8_t gpsLastSatellites = 0;
unsigned long gpsLastFixMs = 0;

void beginGps();
void pollGps();
bool gpsHasFix();
#endif

void connectToWifi();
void sendData();
void beginEnvSensor();
void updateEnvSensor();
bool readHdc1080Sample(float& temperatureC, float& humidityPct);
bool readHdc1080Register(uint8_t reg, uint16_t& value);

void setup() {
  Serial.begin(115200);
  delay(100);

#if defined(ESP32)
  beginGps();
#endif

  beginEnvSensor();
  connectToWifi();

}

void loop() {
#if defined(ESP32)
  pollGps();
#endif

  if (WiFi.status() != WL_CONNECTED) {
    connectToWifi();
  } else {
    sendData();
  }
  updateEnvSensor();
  delay(3000);
}

void connectToWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastWifiAttempt < WIFI_RETRY_MS) {
    return;
  }
  lastWifiAttempt = now;

  Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 40) {
    delay(250);
    Serial.print(".");
    attempt++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WIFI] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WIFI] Connection failed");
  }
}

void sendData(){
  if (WiFi.status() != WL_CONNECTED) return;

String payload;
  payload += "{";
  payload += "\"token\":\"secreto\",";
#if defined(ESP32)
  if (gpsHasFix()) {
    payload += "\"latitud\":"; payload += String(gpsLastLat, 6);
    payload += ",\"longitud\":"; payload += String(gpsLastLon, 6);
  } else {
    payload += "\"latitud\":"; payload += String(0, 6);
    payload += ",\"longitud\":"; payload += String(0, 6);
  }
#endif
  if (envSensorHasSample) {
    payload += ",\"temperatura\":";
    payload += String(envLastTemperatureC, 2);
    payload += ",\"humedad\":";
    payload += String((float)(envLastHumidityPct) / 100, 1);
  } else {
    payload += ",\"temperatura\":";
    payload += String(0, 2);
    payload += ",\"humedad\":";
    payload += String(0, 3);
  }
  payload += "}";

  Serial.printf("[HTTP] -> %s\n", payload.c_str());

if(client.connect(SERVER_URL, 8080)){
  client.println("POST /robot/update_data HTTP/1.1");
  client.print("Host: ");
  client.println(SERVER_URL);
  client.println("User-Agent: Arduino/1.0");
  client.println("Connection: close");
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(payload.length());
  client.println();
  client.print(payload);

  Serial.println("Se envio el mensaje");
  }
else {
    Serial.println("Fallo el envio");
  }
}

void beginEnvSensor() {
#if defined(ESP32)
  Wire.begin(HDC1080_SDA_PIN, HDC1080_SCL_PIN);
#else
  Wire.begin();
#endif

  Wire.beginTransmission(HDC1080_ADDR);
  const bool found = (Wire.endTransmission() == 0);
  envSensorInitialized = found;
  if (found) {
    Serial.println("[HDC1080] Sensor detected");
  } else {
    Serial.println("[HDC1080] Sensor not found on I2C");
  }
}

void updateEnvSensor() {
  if (!envSensorInitialized) {
    return;
  }
  const unsigned long now = millis();
  if (envSensorHasSample && (now - envLastSampleMs) < ENV_SAMPLE_PERIOD_MS) {
    return;
  }

  float temperatureC = 0.0f;
  float humidityPct = 0.0f;
  if (readHdc1080Sample(temperatureC, humidityPct)) {
    envLastTemperatureC = temperatureC;
    envLastHumidityPct = humidityPct;
    envSensorHasSample = true;
  } else {
    Serial.println("[HDC1080] Failed to read sensor");
  }
  envLastSampleMs = now;
}

bool readHdc1080Sample(float& temperatureC, float& humidityPct) {
  uint16_t rawTemp = 0;
  uint16_t rawHumidity = 0;
  if (!readHdc1080Register(0x00, rawTemp)) {
    return false;
  }
  if (!readHdc1080Register(0x01, rawHumidity)) {
    return false;
  }

  temperatureC = (static_cast<float>(rawTemp) / 65536.0f) * 165.0f - 40.0f;
  humidityPct = (static_cast<float>(rawHumidity) / 65536.0f) * 100.0f;
  humidityPct = constrain(humidityPct, 0.0f, 100.0f);
  return true;
}

bool readHdc1080Register(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(HDC1080_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  delay(HDC1080_MEAS_DELAY_MS);
  
  Wire.requestFrom(HDC1080_ADDR, (uint8_t)2);

  if (Wire.available() < 2) {
    while (Wire.available()) Wire.read();
    return false;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  value = (msb << 8) | lsb;
  return true;
}

#if defined(ESP32)
void beginGps() {
  SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[GPS] Serial initialized for TinyGPS++");
}

void pollGps() {
  bool sentenceCompleted = false;
  while (SerialGPS.available()) {
    sentenceCompleted |= gps.encode(SerialGPS.read());
  }

  if (!sentenceCompleted) {
    return;
  }

  const unsigned long now = millis();
  if (gps.location.isValid()) {
    gpsLastLat = gps.location.lat();
    gpsLastLon = gps.location.lng();
    gpsLastFixMs = now;
  }
  if (gps.altitude.isValid()) {
    gpsLastAlt = gps.altitude.meters();
  }
  if (gps.speed.isValid()) {
    gpsLastSpeedMps = gps.speed.mps();
  }
  if (gps.hdop.isValid()) {
    gpsLastHdop = gps.hdop.hdop();
  }
  if (gps.satellites.isValid()) {
    gpsLastSatellites = static_cast<uint8_t>(gps.satellites.value());
  }
}

bool gpsHasFix() {
  const unsigned long age = gps.location.isValid() ? gps.location.age() : GPS_MAX_AGE_MS + 1;
  return gps.location.isValid() && age <= GPS_MAX_AGE_MS;
}
#endif