
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include <DHT.h>

// --- CONFIGURATION CONSTANTS ---
#define WIFI_SSID "TESTESP"
#define WIFI_PASSWORD "12345678"

// Firebase credentials
#define API_KEY "AIzaSyCCYiljdtPktFUmUht59_d8K7v7WeiPWdg"
#define USER_EMAIL "saunok@gmail.com"
#define USER_PASSWORD "12345678"
#define DATABASE_URL "https://control-greenhouse-default-rtdb.asia-southeast1.firebasedatabase.app"

// --- PIN DEFINITIONS ---
#define DHTPIN 14
#define DHTTYPE DHT11
#define SOIL_MOISTURE_PIN 27
#define LDR_SENSOR_PIN 35
#define MQ135_PIN 34
#define RELAY_PIN 25

// --- SENSOR OBJECTS ---
DHT dht(DHTPIN, DHTTYPE);

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 3000;

const unsigned long maxPumpDurationMs = 5000;
const unsigned long minGapBetweenWateringsMs = 30000;
unsigned long lastWaterTime = 0;
unsigned long pumpOnUntil = 0;

float Kp = 2.0f, Ki = 0.08f, Kd = 0.4f;
float prevError = 0.0f, integral = 0.0f;
unsigned long prevPidTime = 0;
const float integralMax = 500.0f;
float moistureSetpoint = 60.0f;

float Kp_fan = 3.0f, Ki_fan = 0.2f, Kd_fan = 0.5f;
float tempSetpoint = 28.0f;
float fanPrevError = 0, fanIntegral = 0;
unsigned long fanPrevPidTime = 0;


UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);
FirebaseApp app;
WiFiClientSecure auth_client;
AsyncClientClass async_auth(auth_client);
WiFiClientSecure db_client;
AsyncClientClass async_db(db_client);
RealtimeDatabase Database;


void processData(AsyncResult &aResult) {
  if (!aResult.isResult()) return;
  if (aResult.isError())
    Serial.printf("[Error] %s\n", aResult.error().message().c_str());
}


void sendPumpState(bool on, unsigned long durationMs) {
  Database.set<bool>(async_db, "/iot/device/pump_on", on, processData);
  Database.set<unsigned long>(async_db, "/iot/device/last_water_duration_ms", durationMs, processData);
}


void fuzzyAdjustGains(float tempC, float humPct, int gasRaw) {
  float baseKp = 2.0f;

  if (!isnan(tempC)) {
    if (tempC >= 32) baseKp += 0.7;
    else if (tempC >= 28) baseKp += 0.4;
    else if (tempC <= 18) baseKp -= 0.4;
  }

  if (!isnan(humPct)) {
    if (humPct >= 75) baseKp -= 0.6;
    else if (humPct >= 60) baseKp -= 0.2;
  }

  float gasPct = map(constrain(gasRaw, 0, 4095), 0, 4095, 0, 100);
  if (gasPct > 70) baseKp -= 0.6;

  Kp = constrain(baseKp, 0.6f, 4.0f);
  Ki = 0.06f * (Kp / 2.0f);
  Kd = 0.35f * (Kp / 2.0f);
}


float computePID(float currentMoisture) {
  unsigned long now = millis();
  float dt = (now - prevPidTime) / 1000.0f;
  if (prevPidTime == 0) dt = 1.0f;

  float error = moistureSetpoint - currentMoisture;
  integral += error * dt;
  integral = constrain(integral, -integralMax, integralMax);

  float derivative = (error - prevError) / dt;
  prevError = error;
  prevPidTime = now;

  return constrain(Kp * error + Ki * integral + Kd * derivative, 0, 100);
}


float computeFanPID(float currentTemp) {
  unsigned long now = millis();
  float dt = (now - fanPrevPidTime) / 1000.0f;
  if (fanPrevPidTime == 0) dt = 1.0f;

  float error = currentTemp - tempSetpoint;
  fanIntegral += error * dt;
  fanIntegral = constrain(fanIntegral, -300, 300);

  float derivative = (error - fanPrevError) / dt;
  fanPrevError = error;
  fanPrevPidTime = now;

  return constrain((Kp_fan * error) + (Ki_fan * fanIntegral) + (Kd_fan * derivative), 0, 100);
}

// Pump ON
void setPumpOnForMs(unsigned long durationMs) {
  if (durationMs == 0) return;
  unsigned long now = millis();

  if (now - lastWaterTime < minGapBetweenWateringsMs) return;

  durationMs = min(durationMs, maxPumpDurationMs);
  pumpOnUntil = now + durationMs;
  lastWaterTime = now;

  digitalWrite(RELAY_PIN, HIGH);
  sendPumpState(true, durationMs);
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(RELAY_PIN, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  auth_client.setInsecure();
  db_client.setInsecure();
  initializeApp(async_auth, app, getAuth(user_auth), processData);
  while (!app.ready()) app.loop();
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);
}

static bool lastPumpState = false;


void loop() {
  app.loop();
   Database.loop();

  // Pump OFF logic
  if (millis() >= pumpOnUntil)
    digitalWrite(RELAY_PIN, LOW);

  bool pumpCurrentlyOn = (millis() < pumpOnUntil);
  if (pumpCurrentlyOn != lastPumpState) {
    lastPumpState = pumpCurrentlyOn;
    sendPumpState(pumpCurrentlyOn, pumpCurrentlyOn ? (pumpOnUntil - millis()) : 0);
  }

  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();

    float temp = dht.readTemperature();
    float hum = dht.readHumidity();

    static float soil = 61;
    soil += random(-4, 5);     // vary -4% to +4%
    soil = constrain(soil, 30, 90);

  
    static int ldr = 2000;
    ldr += random(-200, 201);
    ldr = constrain(ldr, 0, 4095);

    int gas = analogRead(MQ135_PIN);

    fuzzyAdjustGains(temp, hum, gas);

    float pidOutput = computePID(soil);
    Database.set<float>(async_db, "/iot/device/pid_output", pidOutput, processData);

    float fanPID = computeFanPID(temp);
    Database.set<float>(async_db, "/iot/device/fan_pid_output", fanPID, processData);
    Database.set<bool>(async_db, "/iot/device/fan_trigger", fanPID > 20, processData);

    // Save sensor data to Firebase
    Database.set<float>(async_db, "/iot/device/temperature", temp, processData);
    Database.set<float>(async_db, "/iot/device/humidity", hum, processData);
    Database.set<float>(async_db, "/iot/device/soil_pct", soil, processData);
    Database.set<int>(async_db, "/iot/device/light_raw", ldr, processData);
    Database.set<int>(async_db, "/iot/device/gas_raw", gas, processData);

    // Pump activation decision
    unsigned long duration = map(pidOutput, 0, 100, 0, maxPumpDurationMs);
    bool pid_trigger = (soil < moistureSetpoint - 2 && duration > 500);

    Database.set<bool>(async_db, "/iot/device/pid_trigger", pid_trigger, processData);
    if (pid_trigger) setPumpOnForMs(duration);
  }
}
