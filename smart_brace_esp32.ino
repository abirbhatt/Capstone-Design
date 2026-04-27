/*
 * Smart ACL Recovery Brace — ESP32 Firmware
 * ------------------------------------------------
 * Single ESP32 + two MPU6050 IMUs + two load cells via HX711.
 * Tracks knee flexion, valgus, heel load, and toe load.
 * Broadcasts all data over WebSocket at 20 Hz.
 *
 * Required libraries:
 *   • WebSockets   (by Markus Sattler)
 *   • Adafruit MPU6050
 *   • Adafruit Unified Sensor
 *   • HX711        (by Bogdan Necula / Rob Tillaart)
 *
 * ── TUNING ────────────────────────────────────────────────────────────────────
 * If flexion reads backwards:  flip UPPER_PITCH_SIGN or LOWER_PITCH_SIGN to -1
 * If valgus reads backwards:   flip UPPER_ROLL_SIGN  or LOWER_ROLL_SIGN  to -1
 * If heel/toe signs are wrong: negate CALIB_HEEL or CALIB_TOE
 * ─────────────────────────────────────────────────────────────────────────────
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

// HX711 pins (injured leg only — healthy leg added later)
const int HX711_DT  = 18;
const int HX711_SCK = 19;

// Load cell calibration factors
// Channel A (gain 128) = heel,  Channel B (gain 32) = toe
const float CALIB_HEEL = 43801.0f;
const float CALIB_TOE  = 14072.0f;

// Safety thresholds
// Valgus alert at 10° — catches deliberate knee cave without false alarms on
// clean bends (which read ~3° after coupling compensation).
const float VALGUS_ALERT_DEG  = 10.0f;
const float FLEXION_ALERT_DEG = 100.0f;

// Valgus coupling compensation (~0.17 deg of apparent valgus per deg of flex).
// Empirically derived from CSV data. Re-derive if IMUs are remounted.
const float VALGUS_COUPLING_K = 0.17f;

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

// ── LOAD CELL STATE ───────────────────────────────────────────────────────────
float heel_kg     = 0.0f;
float toe_kg      = 0.0f;
long  offset_heel = 0;
long  offset_toe  = 0;
bool  cells_ready = false;

// Non-blocking read state machine
// Each call to updateLoadCells() does exactly ONE scale.read() only when
// the HX711 signals data is ready (DT pin LOW). This never blocks the loop.
//
// Cycle: COMMIT_HEEL → READ_HEEL → COMMIT_TOE → READ_TOE → repeat
//   COMMIT phases do a throwaway read that switches gain for the next channel.
//   READ phases capture the real measurement.
// At 10 SPS: full cycle takes ~400ms. At 80 SPS (RATE pin HIGH): ~50ms.
enum LCPhase { LC_COMMIT_HEEL, LC_READ_HEEL, LC_COMMIT_TOE, LC_READ_TOE };
LCPhase lcPhase = LC_COMMIT_HEEL;

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
  // Subtract artifact caused by lower IMU rotating during knee flexion
  float coupling = VALGUS_COUPLING_K * getKneeFlexion();
  return raw - coupling;
}

// ── LOAD CELL — TARE (blocking, only called at startup / on demand) ───────────
void tareCells(int samples = 20) {
  Serial.println("Taring load cells — keep foot flat and still...");
  long sumH = 0, sumT = 0;

  for (int i = 0; i < samples; i++) {
    scale.set_gain(128); scale.read(); sumH += scale.read();   // heel
    scale.set_gain(32);  scale.read(); sumT += scale.read();   // toe
  }
  offset_heel = sumH / samples;
  offset_toe  = sumT / samples;
  lcPhase = LC_COMMIT_HEEL;   // reset state machine after tare

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
// Called every loop iteration. Does nothing unless HX711 has data ready.
// One call = one scale.read() = microseconds when DT is already LOW.
void updateLoadCells() {
  if (!cells_ready || !scale.is_ready()) return;

  switch (lcPhase) {
    case LC_COMMIT_HEEL:
      // Throwaway read that commits gain 128 (heel) for the next conversion
      scale.set_gain(128);
      scale.read();
      lcPhase = LC_READ_HEEL;
      break;

    case LC_READ_HEEL:
      heel_kg = (scale.read() - offset_heel) / CALIB_HEEL;
      lcPhase = LC_COMMIT_TOE;
      break;

    case LC_COMMIT_TOE:
      // Throwaway read that commits gain 32 (toe) for the next conversion
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

// ── BROADCAST ─────────────────────────────────────────────────────────────────
void broadcastSensorData() {
  float flexion  = getKneeFlexion();
  float valgus   = getKneeValgus();
  float total_kg = heel_kg + toe_kg;
  bool  alert    = (flexion > FLEXION_ALERT_DEG) ||
                   (fabsf(valgus) > VALGUS_ALERT_DEG);

  char buf[320];
  snprintf(buf, sizeof(buf),
    "{"
    "\"knee_angle\":%.1f,"
    "\"valgus\":%.1f,"
    "\"upper_pitch\":%.1f,\"upper_roll\":%.1f,"
    "\"lower_pitch\":%.1f,\"lower_roll\":%.1f,"
    "\"heel\":%.2f,\"toe\":%.2f,\"load_total\":%.2f,"
    "\"alert\":%s,\"ts\":%lu"
    "}",
    flexion, valgus,
    UPPER_PITCH_SIGN * (stateUpper.pitch - zeroUpper.pitch),
    UPPER_ROLL_SIGN  * (stateUpper.roll  - zeroUpper.roll),
    LOWER_PITCH_SIGN * (stateLower.pitch - zeroLower.pitch),
    LOWER_ROLL_SIGN  * (stateLower.roll  - zeroLower.roll),
    heel_kg, toe_kg, total_kg,
    alert ? "true" : "false",
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
      break;
    case WStype_TEXT:
      if (strcmp((char*)payload, "calibrate") == 0) {
        Serial.println("[WS] IMU recalibration requested");
        calibrateZeroOffsets();
        webSocket.sendTXT(clientNum, "{\"calibrated\":true}");
      }
      else if (strcmp((char*)payload, "tare") == 0) {
        Serial.println("[WS] Load cell tare requested");
        tareCells();
        webSocket.sendTXT(clientNum, "{\"tared\":true}");
      }
      break;
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
  Serial.println("\n-- Smart ACL Brace Booting --");

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

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.printf("AP started — SSID: %s  IP: %s\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str());

  // WebSocket
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("WebSocket ready on port 80\n");
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

  // Broadcast at 20 Hz
  if (millis() - lastBroadcast >= BROADCAST_MS) {
    broadcastSensorData();
    lastBroadcast = millis();
  }

  delay(1);
}
