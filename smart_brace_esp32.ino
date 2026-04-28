/*
 * Smart ACL Recovery Brace — Main (Injured Leg) ESP32
 * -------------------------------------------------------
 * Runs as a WiFi Access Point ("ACL_Brace").
 * Two MPU6050 IMUs track knee flexion and valgus.
 * One HX711 reads heel + toe load cells on the injured leg.
 * The healthy-leg ESP32 joins this AP as a WebSocket client
 * and sends {"healthy":1,...} packets which are merged into
 * every broadcast so the dashboard sees both legs + symmetry.
 *
 * Required libraries:
 *   • WebSockets         (by Markus Sattler)
 *   • Adafruit MPU6050
 *   • Adafruit Unified Sensor
 *   • HX711              (by Bogdan Necula / Rob Tillaart)
 *   • Adafruit NeoPixel
 *
 * ── TUNING ──────────────────────────────────────────────────────────────────
 * If flexion reads backwards:  flip UPPER_PITCH_SIGN or LOWER_PITCH_SIGN to -1
 * If valgus reads backwards:   flip UPPER_ROLL_SIGN  or LOWER_ROLL_SIGN  to -1
 * If heel/toe signs are wrong: negate CALIB_HEEL or CALIB_TOE
 * ────────────────────────────────────────────────────────────────────────────
 */

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HX711.h>
// ── USER CONFIGURATION ────────────────────────────────────────────────────────
const char* WIFI_SSID = "ACL_Brace";
const char* WIFI_PASS = "aclbrace1";

// Flip any of these to -1.0f if an axis reads backwards
const float UPPER_PITCH_SIGN =  1.0f;
const float LOWER_PITCH_SIGN =  1.0f;
const float UPPER_ROLL_SIGN  =  1.0f;
const float LOWER_ROLL_SIGN  =  1.0f;

// I2C + IMU addresses
const int     I2C_SDA        = 21;
const int     I2C_SCL        = 22;
const uint8_t MPU_UPPER_ADDR = 0x68;   // AD0 LOW  (GND)
const uint8_t MPU_LOWER_ADDR = 0x69;   // AD0 HIGH (3.3V)

// HX711 pins (injured leg)
const int HX711_DT  = 18;
const int HX711_SCK = 19;

// Load cell calibration factors
// Channel A (gain 128) = heel,  Channel B (gain 32) = toe
const float CALIB_HEEL = 43801.0f;
const float CALIB_TOE  = 14072.0f;

// Safety thresholds
const float VALGUS_ALERT_DEG  = 10.0f;
const float FLEXION_ALERT_DEG = 90.0f;

// Valgus coupling compensation — empirically derived from CSV data
const float VALGUS_COUPLING_K = 0.17f;

// Symmetry threshold: alert when injured/healthy symmetry drops below this
const float SYMMETRY_ALERT_PCT = 70.0f;

// How long without a healthy-leg packet before we consider it disconnected
const uint32_t HEALTHY_TIMEOUT_MS = 2000;

// Broadcast interval
const uint32_t BROADCAST_MS = 50;    // 20 Hz

// ─────────────────────────────────────────────────────────────────────────────

WebSocketsServer webSocket(80);
Adafruit_MPU6050 mpuUpper;
Adafruit_MPU6050 mpuLower;
HX711            scale;

// ── IMU STATE ─────────────────────────────────────────────────────────────────
struct ImuState {
  float pitch  = 0.0f;
  float roll   = 0.0f;
  unsigned long lastMs = 0;
};

struct ZeroOffset {
  float pitch = 0.0f;
  float roll  = 0.0f;
};

ImuState   stateUpper, stateLower;
ZeroOffset zeroUpper,  zeroLower;

// ── HAPTICS GLOBALS (defined in safety_outputs.ino) ───────────────────────────
// Forward-declared here because Arduino concatenates the main .ino first,
// so safety_outputs.ino globals would otherwise be unseen inside loop().
extern float kneeBendAngle;
extern float kneeInwardAngle;
extern float leftLoadCellValue;
extern float rightLoadCellValue;

// ── LOAD CELL STATE ───────────────────────────────────────────────────────────
float heel_kg     = 0.0f;
float toe_kg      = 0.0f;
long  offset_heel = 0;
long  offset_toe  = 0;
bool  cells_ready = false;

enum LCPhase { LC_COMMIT_HEEL, LC_READ_HEEL, LC_COMMIT_TOE, LC_READ_TOE };
LCPhase lcPhase = LC_COMMIT_HEEL;

// ── HEALTHY LEG STATE ─────────────────────────────────────────────────────────
// Updated whenever the healthy-leg ESP32 sends a {"healthy":1,...} packet.
// healthyClientNum tracks which WS client slot is the healthy board so we can
// relay calibrate/tare commands to it directly.
struct HealthyLeg {
  float    heel_kg    = 0.0f;
  float    toe_kg     = 0.0f;
  float    load_total = 0.0f;
  float    knee_angle = 0.0f;
  float    valgus     = 0.0f;
  uint32_t lastSeen   = 0;        // millis() of last received packet
} healthy;

uint8_t healthyClientNum = 255;   // 255 = not yet identified

// ── JSON FIELD EXTRACTOR ──────────────────────────────────────────────────────
// Finds "key":value in a flat JSON string and returns the float value.
// No dynamic allocation — safe for tight embedded loops.
float extractField(const char* json, const char* key) {
  char search[48];
  snprintf(search, sizeof(search), "\"%s\":", key);
  const char* p = strstr(json, search);
  if (!p) return 0.0f;
  p += strlen(search);
  while (*p == ' ') p++;
  return atof(p);
}

// ── IMU FILTER ────────────────────────────────────────────────────────────────
void updateFilteredAngles(Adafruit_MPU6050 &mpu, ImuState &s) {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float accelPitch = atan2(ay, sqrt(ax*ax + az*az)) * RAD_TO_DEG;
  float accelRoll  = atan2(ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

  unsigned long now = millis();
  float dt = (s.lastMs == 0) ? 0.0f : (now - s.lastMs) / 1000.0f;
  s.lastMs = now;

  s.pitch = 0.98f * (s.pitch + gyro.gyro.x * RAD_TO_DEG * dt) + 0.02f * accelPitch;
  s.roll  = 0.98f * (s.roll  + gyro.gyro.y * RAD_TO_DEG * dt) + 0.02f * accelRoll;
}

// ── IMU CALIBRATION ───────────────────────────────────────────────────────────
void calibrateZeroOffsets() {
  const int SAMPLES = 200;
  float sumUpperPitch = 0, sumUpperRoll = 0;
  float sumLowerPitch = 0, sumLowerRoll = 0;

  Serial.println("Calibrating IMUs — hold brace still in standing position...");

  for (int i = 0; i < SAMPLES; i++) {
    updateFilteredAngles(mpuUpper, stateUpper);
    updateFilteredAngles(mpuLower, stateLower);
    sumUpperPitch += stateUpper.pitch;
    sumUpperRoll  += stateUpper.roll;
    sumLowerPitch += stateLower.pitch;
    sumLowerRoll  += stateLower.roll;
    delay(10);
  }

  zeroUpper.pitch = sumUpperPitch / SAMPLES;
  zeroUpper.roll  = sumUpperRoll  / SAMPLES;
  zeroLower.pitch = sumLowerPitch / SAMPLES;
  zeroLower.roll  = sumLowerRoll  / SAMPLES;

  Serial.printf("IMU zero set — Upper pitch: %.1f  roll: %.1f\n",
                zeroUpper.pitch, zeroUpper.roll);
  Serial.printf("               Lower pitch: %.1f  roll: %.1f\n",
                zeroLower.pitch, zeroLower.roll);
}

// ── COMPUTED IMU ANGLES ───────────────────────────────────────────────────────
float getKneeFlexion() {
  float u = UPPER_PITCH_SIGN * (stateUpper.pitch - zeroUpper.pitch);
  float l = LOWER_PITCH_SIGN * (stateLower.pitch - zeroLower.pitch);
  return fabsf(u - l);
}

float getKneeValgus() {
  float u   = UPPER_ROLL_SIGN * (stateUpper.roll - zeroUpper.roll);
  float l   = LOWER_ROLL_SIGN * (stateLower.roll - zeroLower.roll);
  float raw = l - u;
  return raw - VALGUS_COUPLING_K * getKneeFlexion();
}

// ── LOAD CELL — TARE ─────────────────────────────────────────────────────────
void tareCells(int samples = 20) {
  Serial.println("Taring load cells — keep foot flat and still...");
  long sumH = 0, sumT = 0;

  for (int i = 0; i < samples; i++) {
    scale.set_gain(128); scale.read(); sumH += scale.read();
    scale.set_gain(32);  scale.read(); sumT += scale.read();
  }
  offset_heel = sumH / samples;
  offset_toe  = sumT / samples;
  lcPhase = LC_COMMIT_HEEL;

  Serial.printf("Tare done — heel offset: %ld  toe offset: %ld\n",
                offset_heel, offset_toe);
}

void initLoadCells() {
  scale.begin(HX711_DT, HX711_SCK);

  unsigned long start = millis();
  while (!scale.is_ready() && millis() - start < 3000) delay(50);

  if (scale.is_ready()) {
    delay(200);
    tareCells();
    cells_ready = true;
    Serial.println("Load cells: ready");
  } else {
    Serial.println("Load cells: NOT FOUND — check DT/SCK wiring");
  }
}

// ── LOAD CELL — NON-BLOCKING UPDATE ──────────────────────────────────────────
void updateLoadCells() {
  if (!cells_ready || !scale.is_ready()) return;

  switch (lcPhase) {
    case LC_COMMIT_HEEL:
      scale.set_gain(128);
      scale.read();
      lcPhase = LC_READ_HEEL;
      break;
    case LC_READ_HEEL:
      heel_kg = (scale.read() - offset_heel) / CALIB_HEEL;
      lcPhase = LC_COMMIT_TOE;
      break;
    case LC_COMMIT_TOE:
      scale.set_gain(32);
      scale.read();
      lcPhase = LC_READ_TOE;
      break;
    case LC_READ_TOE:
      toe_kg = (scale.read() - offset_toe) / CALIB_TOE;
      lcPhase = LC_COMMIT_HEEL;
      break;
  }
}

// ── PARSE HEALTHY LEG PACKET ──────────────────────────────────────────────────
void parseHealthyData(uint8_t clientNum, const char* json) {
  healthy.knee_angle = extractField(json, "knee_angle");
  healthy.valgus     = extractField(json, "valgus");
  healthy.heel_kg    = extractField(json, "heel_kg");
  healthy.toe_kg     = extractField(json, "toe_kg");
  healthy.load_total = extractField(json, "load_total");
  healthy.lastSeen   = millis();
  healthyClientNum   = clientNum;   // remember which slot for command relay
}

// ── BROADCAST ─────────────────────────────────────────────────────────────────
void broadcastSensorData() {
  float flexion  = getKneeFlexion();
  float valgus   = getKneeValgus();
  float total_kg = heel_kg + toe_kg;
  bool  alert    = (flexion > FLEXION_ALERT_DEG) ||
                   (fabsf(valgus) > VALGUS_ALERT_DEG);

  // Decide whether healthy leg data is fresh enough to use
  bool  hFresh   = (millis() - healthy.lastSeen < HEALTHY_TIMEOUT_MS);
  float hHeel    = hFresh ? healthy.heel_kg    : 0.0f;
  float hToe     = hFresh ? healthy.toe_kg     : 0.0f;
  float hTotal   = hFresh ? healthy.load_total : 0.0f;

  // Symmetry: min/max ratio so 100% = equal load regardless of which leg is heavier
  float symPct   = -1.0f;
  bool  symAlert = false;
  if (hFresh && total_kg > 0.5f && hTotal > 0.5f) {
    float mn = min(total_kg, hTotal);
    float mx = max(total_kg, hTotal);
    symPct   = (mn / mx) * 100.0f;
    symAlert = symPct < SYMMETRY_ALERT_PCT;
  }

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{"
    "\"knee_angle\":%.1f,"
    "\"valgus\":%.1f,"
    "\"upper_pitch\":%.1f,\"upper_roll\":%.1f,"
    "\"lower_pitch\":%.1f,\"lower_roll\":%.1f,"
    "\"heel\":%.2f,\"toe\":%.2f,\"load_total\":%.2f,"
    "\"heel_h\":%.2f,\"toe_h\":%.2f,\"load_total_h\":%.2f,"
    "\"symmetry\":%.1f,"
    "\"sym_alert\":%s,"
    "\"h_connected\":%s,"
    "\"alert\":%s,"
    "\"ts\":%lu"
    "}",
    flexion, valgus,
    UPPER_PITCH_SIGN * (stateUpper.pitch - zeroUpper.pitch),
    UPPER_ROLL_SIGN  * (stateUpper.roll  - zeroUpper.roll),
    LOWER_PITCH_SIGN * (stateLower.pitch - zeroLower.pitch),
    LOWER_ROLL_SIGN  * (stateLower.roll  - zeroLower.roll),
    heel_kg, toe_kg, total_kg,
    hHeel, hToe, hTotal,
    symPct,
    symAlert ? "true" : "false",
    hFresh   ? "true" : "false",
    alert    ? "true" : "false",
    millis());

  webSocket.broadcastTXT(buf);
}

// ── WEBSOCKET EVENTS ──────────────────────────────────────────────────────────
void onWebSocketEvent(uint8_t clientNum, WStype_t type,
                      uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u connected\n", clientNum);
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", clientNum);
      // If healthy leg dropped, clear its slot
      if (clientNum == healthyClientNum) {
        healthyClientNum    = 255;
        healthy.lastSeen    = 0;
        Serial.println("[WS] Healthy leg disconnected");
      }
      break;

    case WStype_TEXT: {
      const char* msg = (const char*)payload;

      // Healthy-leg data packet — identified by "healthy":1 prefix
      if (strncmp(msg, "{\"healthy\"", 10) == 0) {
        parseHealthyData(clientNum, msg);
        break;
      }

      // Browser command: calibrate
      if (strcmp(msg, "calibrate") == 0) {
        Serial.println("[WS] IMU recalibration requested");
        calibrateZeroOffsets();
        webSocket.sendTXT(clientNum, "{\"calibrated\":true}");
        // Relay to healthy leg so both boards zero at the same time
        if (healthyClientNum != 255) {
          webSocket.sendTXT(healthyClientNum, "calibrate");
        }
        break;
      }

      // Browser command: tare load cells
      if (strcmp(msg, "tare") == 0) {
        Serial.println("[WS] Load cell tare requested");
        tareCells();
        webSocket.sendTXT(clientNum, "{\"tared\":true}");
        // Relay to healthy leg
        if (healthyClientNum != 255) {
          webSocket.sendTXT(healthyClientNum, "tare");
        }
        break;
      }

      break;
    }
    default:
      break;
  }
}

// ── IMU SETUP HELPER ──────────────────────────────────────────────────────────
void setupIMU(Adafruit_MPU6050 &mpu, const char* name, uint8_t addr) {
  if (!mpu.begin(addr, &Wire)) {
    Serial.printf("ERROR: %s not found at 0x%02X — check wiring\n", name, addr);
    while (1) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.printf("%s ready at 0x%02X\n", name, addr);
}

// ── SETUP ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n-- Smart ACL Brace (Main) Booting --");

  // IMUs
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  setupIMU(mpuUpper, "Upper IMU", MPU_UPPER_ADDR);
  setupIMU(mpuLower, "Lower IMU", MPU_LOWER_ADDR);
  stateUpper.lastMs = millis();
  stateLower.lastMs = millis();
  calibrateZeroOffsets();

  // Load cells
  initLoadCells();

  // Haptic feedback and LED
  setupSafetyOutputs();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.printf("AP started — SSID: %s  IP: %s\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str());

  // WebSocket server — browser AND healthy ESP32 both connect here
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("WebSocket server ready on port 80");
  Serial.println("Waiting for browser and/or healthy-leg ESP32...\n");
}

// ── LOOP ──────────────────────────────────────────────────────────────────────
uint32_t lastBroadcast = 0;

void loop() {
  webSocket.loop();

  // IMU — every iteration for smooth complementary filter
  updateFilteredAngles(mpuUpper, stateUpper);
  updateFilteredAngles(mpuLower, stateLower);

  // Load cells — non-blocking, self-throttles via HX711 DRDY signal
  updateLoadCells();

  // Feed latest sensor readings into haptics globals, then update outputs.
  // This runs every iteration so motors and LED respond with minimal latency.
  kneeBendAngle      = getKneeFlexion();
  kneeInwardAngle    = fabsf(getKneeValgus());
  leftLoadCellValue  = heel_kg;
  rightLoadCellValue = toe_kg;
  updateSafetyOutputs();

  // Broadcast at 20 Hz
  if (millis() - lastBroadcast >= BROADCAST_MS) {
    broadcastSensorData();
    lastBroadcast = millis();
  }

  delay(1);
}
