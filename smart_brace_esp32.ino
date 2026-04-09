/*
 * Smart ACL Recovery Brace — ESP32 Firmware
 * ─────────────────────────────────────────
 * Sets up a WiFi WebSocket server and broadcasts sensor data at 20 Hz.
 * Simulated readings are used until real sensors are wired in.
 * Each placeholder function is clearly marked with TODO.
 *
 * Required libraries (install via Arduino Library Manager):
 *   • ESPAsyncWebServer  (by lacamera / me-no-dev)
 *   • AsyncTCP           (by dvarrel / me-no-dev) — dependency of above
 *   • ArduinoJson        (by Benoit Blanchon)
 *
 * ── SETUP STEPS ──────────────────────────────────────────────────────────────
 *  1. Set WIFI_SSID and WIFI_PASS below to your network.
 *  2. Set THIS_LEG to "injured" or "healthy" depending on which brace this is.
 *  3. Flash to ESP32. Open Serial Monitor (115200 baud).
 *  4. Note the IP address printed — enter it in dashboard.html when prompted.
 * ─────────────────────────────────────────────────────────────────────────────
 */

//  Different modes: 
//  Walking mode (if knee angle is not too large)
//  Squatting mode (if knee angle is large and load cell data is high)
//  Endless loop that automatically detects which mode user should be in and updates UI

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
#include <math.h>

// ── USER CONFIGURATION ────────────────────────────────────────────────────────
const char* WIFI_SSID  = "AB PHONE";
const char* WIFI_PASS  = "12345678";
const char* THIS_LEG   = "injured";   // Change to "healthy" on the other ESP32

// ── SAFETY THRESHOLDS ─────────────────────────────────────────────────────────
const float ANGLE_ALERT_HIGH = 100.0;  // degrees — haptic fires above this
const float ANGLE_ALERT_LOW  = -5.0;   // degrees — haptic fires below this

// ── BROADCAST RATE ────────────────────────────────────────────────────────────
const uint32_t BROADCAST_MS = 50;   // 20 Hz

// ── PIN DEFINITIONS ───────────────────────────────────────────────────────────
// Only used on the injured-leg ESP32
#define PIN_HAPTIC  4    // Coin vibration motor (via transistor/MOSFET)
#define PIN_LED     2    // WS2812B data pin  (or onboard LED for testing)

// ─────────────────────────────────────────────────────────────────────────────

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── SENSOR PLACEHOLDERS ───────────────────────────────────────────────────────
// These functions return simulated data now.
// When your sensors are wired up, replace the bodies with real reads.

/**
 * getKneeAngle()
 * Returns: Knee flexion angle in degrees (0 = fully extended, 90 = right angle).
 *
 * TODO: Replace with complementary filter output from two IMUs.
 *       thighAngle = complementaryFilter(thigh_imu);
 *       shinAngle  = complementaryFilter(shin_imu);
 *       return thighAngle - shinAngle;  // relative angle between segments
 *
 * Sim: Oscillates 5°–90° every ~6 seconds (like slow leg extensions).
 */
float getKneeAngle() {
  float t = millis() / 6000.0;
  return 47.5 + 42.5 * sin(t * 2.0 * PI);
}

/**
 * getLoadKg()
 * Returns: Ground reaction force in kg through this leg's load cell.
 *
 * TODO: Replace with HX711 read once load cells arrive.
 *       scale.wait_ready();
 *       long raw = scale.read();
 *       return (raw - TARE_OFFSET) / CALIBRATION_FACTOR;
 *
 * Sim: Returns 0 (signals "no sensor") so dashboard shows "---".
 */
float getLoadKg() {
  return 0.0;   // ← change to real read when HX711 is wired
}

// ── COMPLEMENTARY FILTER STATE ────────────────────────────────────────────────
// Included here so you can paste your IMU logic directly.
// Each IMU needs its own instance of these variables.

float thighAngle = 0.0;
float shinAngle  = 0.0;
unsigned long lastImuTime = 0;

/**
 * updateIMU()
 * TODO: Call this in loop() once IMU is wired.
 *       Replace the placeholder values with actual accel/gyro reads.
 *
 * Example (MPU6050 / ICM-42688-P via I2C):
 *   float ax, ay, az, gx;   // read from your IMU library
 *   float dt = (millis() - lastImuTime) / 1000.0;
 *   float accelAngle = atan2(ay, az) * 180.0 / PI;
 *   thighAngle = 0.98 * (thighAngle + gx * dt) + 0.02 * accelAngle;
 *   lastImuTime = millis();
 *   // Repeat for shin IMU → shinAngle
 *   // kneeAngle = thighAngle - shinAngle;
 */
void updateIMU() {
  // TODO: implement once hardware is ready
}

// ─────────────────────────────────────────────────────────────────────────────

bool isUnsafe(float angle) {
  return (angle > ANGLE_ALERT_HIGH) || (angle < ANGLE_ALERT_LOW);
}

/**
 * triggerFeedback()
 * Only fires on the injured-leg ESP32 — independent of network.
 */
void triggerFeedback(bool alert) {
  if (strcmp(THIS_LEG, "injured") == 0) {
    digitalWrite(PIN_HAPTIC, alert ? HIGH : LOW);
    digitalWrite(PIN_LED,    alert ? HIGH : LOW);
  }
}

// ── BROADCAST ─────────────────────────────────────────────────────────────────
void broadcastSensorData() {
  float angle = getKneeAngle();
  float load  = getLoadKg();
  bool  alert = isUnsafe(angle);

  triggerFeedback(alert);

  // Build JSON packet
  StaticJsonDocument<128> doc;
  doc["leg"]         = THIS_LEG;
  doc["knee_angle"]  = round(angle * 10.0) / 10.0;   // 1 decimal place
  doc["load_kg"]     = round(load  * 10.0) / 10.0;
  doc["alert"]       = alert;
  doc["ts"]          = millis();

  char buf[128];
  serializeJson(doc, buf);
  ws.textAll(buf);
}

// ── SETUP ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n── Smart ACL Brace Booting ──");

  pinMode(PIN_HAPTIC, OUTPUT);
  pinMode(PIN_LED,    OUTPUT);
  digitalWrite(PIN_HAPTIC, LOW);
  digitalWrite(PIN_LED,    LOW);

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println(" connected!");
  Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
  Serial.println("→ Enter this IP in dashboard.html when prompted");
  Serial.printf("Leg: %s\n\n", THIS_LEG);

  // WebSocket event handler
  ws.onEvent([](AsyncWebSocket* s, AsyncWebSocketClient* c,
                AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT)
      Serial.printf("[WS] Client #%u connected from %s\n",
                    c->id(), c->remoteIP().toString().c_str());
    else if (type == WS_EVT_DISCONNECT)
      Serial.printf("[WS] Client #%u disconnected\n", c->id());
  });

  server.addHandler(&ws);
  server.begin();
  Serial.println("WebSocket server started → ws://<IP>/ws");
}

// ── LOOP ──────────────────────────────────────────────────────────────────────
uint32_t lastBroadcast = 0;

void loop() {
  if (millis() - lastBroadcast >= BROADCAST_MS) {
    broadcastSensorData();
    ws.cleanupClients();   // free memory from dropped connections
    lastBroadcast = millis();
  }

  // TODO: Call updateIMU() here once hardware is wired
  // updateIMU();
}
