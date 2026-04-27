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

// HX711 pins — no conflict with I2C (21/22)
// Each HX711 needs its own dedicated SCK (shared SCK corrupts gain switching)
const int HX711_DT      = 18;   // injured leg DT
const int HX711_SCK     = 19;   // injured leg SCK
const int HX711_DT_H    =  5;   // healthy leg DT
const int HX711_SCK_H   = 17;   // healthy leg SCK

// Load cell calibration factors (from teammate's measurements)
// HX711 Channel A (gain 128) = heel,  Channel B (gain 32) = toe
const float CALIB_HEEL   = 43801.0f;   // injured leg
const float CALIB_TOE    = 14072.0f;   // injured leg
const float CALIB_HEEL_H = 40136.0f;   // healthy leg
const float CALIB_TOE_H  = 24300.0f;   // healthy leg (sign flipped vs teammate's -24300)

// Safety thresholds
const float VALGUS_ALERT_DEG  = 15.0f;
const float FLEXION_ALERT_DEG = 100.0f;

// Valgus coupling compensation (~0.17 deg of apparent valgus per deg of flex).
// Empirically derived from CSV data. Re-derive if IMUs are remounted.
const float VALGUS_COUPLING_K = 0.17f;

// Timing
const uint32_t BROADCAST_MS   =  50;   // 20 Hz WebSocket broadcast
const uint32_t LOAD_UPDATE_MS = 200;   // 5 Hz load cell reads (HX711 is slow)

// ─────────────────────────────────────────────────────────────────────────────

WebSocketsServer webSocket(80);
Adafruit_MPU6050 mpuUpper;
Adafruit_MPU6050 mpuLower;
HX711            scale;        // injured leg
HX711            scaleHealthy; // healthy leg

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
// Injured leg
float heel_kg     = 0.0f;
float toe_kg      = 0.0f;
long  offset_heel = 0;
long  offset_toe  = 0;
bool  cells_ready = false;

// Healthy leg
float heel_kg_h     = 0.0f;
float toe_kg_h      = 0.0f;
long  offset_heel_h = 0;
long  offset_toe_h  = 0;
bool  cells_ready_h = false;

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

// ── LOAD CELL HELPERS ─────────────────────────────────────────────────────────
// HX711 multiplexes two load cells via gain switching:
//   Gain 128 -> Channel A -> heel load cell
//   Gain  32 -> Channel B -> toe  load cell
// Each gain switch requires one throwaway read before the real read.
// NOTE: Tie each HX711 RATE pin to 3.3V for 80 SPS mode.
//       Floating/GND = 10 SPS — each read blocks ~100 ms.

// ── Injured leg reads ──────────────────────────────────────────────────────
long readHeelRaw() {
  scale.set_gain(128);
  scale.read();
  return scale.read();
}
long readToeRaw() {
  scale.set_gain(32);
  scale.read();
  return scale.read();
}

void tareCells(int samples = 20) {
  long sumH = 0, sumT = 0;
  Serial.println("Taring injured leg — keep foot flat...");
  for (int i = 0; i < samples; i++) {
    sumH += readHeelRaw();
    sumT += readToeRaw();
  }
  offset_heel = sumH / samples;
  offset_toe  = sumT / samples;
  Serial.printf("Injured tare done — heel: %ld  toe: %ld\n",
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
    Serial.println("Injured leg load cells: ready");
  } else {
    Serial.println("Injured leg load cells: NOT FOUND");
  }
}

// ── Healthy leg reads ──────────────────────────────────────────────────────
long readHeelRawH() {
  scaleHealthy.set_gain(128);
  scaleHealthy.read();
  return scaleHealthy.read();
}
long readToeRawH() {
  scaleHealthy.set_gain(32);
  scaleHealthy.read();
  return scaleHealthy.read();
}

void tareCellsHealthy(int samples = 20) {
  long sumH = 0, sumT = 0;
  Serial.println("Taring healthy leg — keep foot flat...");
  for (int i = 0; i < samples; i++) {
    sumH += readHeelRawH();
    sumT += readToeRawH();
  }
  offset_heel_h = sumH / samples;
  offset_toe_h  = sumT / samples;
  Serial.printf("Healthy tare done — heel: %ld  toe: %ld\n",
                offset_heel_h, offset_toe_h);
}

void initLoadCellsHealthy() {
  scaleHealthy.begin(HX711_DT_H, HX711_SCK_H);
  unsigned long start = millis();
  while (!scaleHealthy.is_ready() && millis() - start < 3000) delay(50);

  if (scaleHealthy.is_ready()) {
    delay(200);
    tareCellsHealthy();
    cells_ready_h = true;
    Serial.println("Healthy leg load cells: ready");
  } else {
    Serial.println("Healthy leg load cells: NOT FOUND");
  }
}

// ── Combined update ────────────────────────────────────────────────────────
void updateLoadCells() {
  if (cells_ready && scale.is_ready()) {
    heel_kg = (readHeelRaw() - offset_heel) / CALIB_HEEL;
    toe_kg  = (readToeRaw()  - offset_toe)  / CALIB_TOE;
  }
  if (cells_ready_h && scaleHealthy.is_ready()) {
    heel_kg_h = (readHeelRawH() - offset_heel_h) / CALIB_HEEL_H;
    toe_kg_h  = (readToeRawH()  - offset_toe_h)  / CALIB_TOE_H;
  }
}

// ── BROADCAST ─────────────────────────────────────────────────────────────────
void broadcastSensorData() {
  float flexion    = getKneeFlexion();
  float valgus     = getKneeValgus();
  float total_i    = heel_kg   + toe_kg;    // injured leg total
  float total_h    = heel_kg_h + toe_kg_h;  // healthy leg total
  bool  alert      = (flexion > FLEXION_ALERT_DEG) ||
                     (fabsf(valgus) > VALGUS_ALERT_DEG);

  char buf[420];
  snprintf(buf, sizeof(buf),
    "{"
    "\"knee_angle\":%.1f,"
    "\"valgus\":%.1f,"
    "\"upper_pitch\":%.1f,\"upper_roll\":%.1f,"
    "\"lower_pitch\":%.1f,\"lower_roll\":%.1f,"
    "\"heel\":%.2f,\"toe\":%.2f,\"load_total\":%.2f,"
    "\"heel_h\":%.2f,\"toe_h\":%.2f,\"load_total_h\":%.2f,"
    "\"alert\":%s,\"ts\":%lu"
    "}",
    flexion, valgus,
    UPPER_PITCH_SIGN * (stateUpper.pitch - zeroUpper.pitch),
    UPPER_ROLL_SIGN  * (stateUpper.roll  - zeroUpper.roll),
    LOWER_PITCH_SIGN * (stateLower.pitch - zeroLower.pitch),
    LOWER_ROLL_SIGN  * (stateLower.roll  - zeroLower.roll),
    heel_kg, toe_kg, total_i,
    heel_kg_h, toe_kg_h, total_h,
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
        Serial.println("[WS] Load cell tare requested — both legs");
        tareCells();
        tareCellsHealthy();
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

  // Load cells — injured then healthy
  initLoadCells();
  initLoadCellsHealthy();

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
uint32_t lastBroadcast  = 0;
uint32_t lastLoadUpdate = 0;

void loop() {
  webSocket.loop();

  // IMU — update every iteration for a smooth complementary filter
  updateFilteredAngles(mpuUpper, stateUpper);
  updateFilteredAngles(mpuLower, stateLower);

  // Load cells — update at 5 Hz; each read pair blocks ~25 ms at 80 SPS
  if (millis() - lastLoadUpdate >= LOAD_UPDATE_MS) {
    updateLoadCells();
    lastLoadUpdate = millis();
  }

  // Broadcast at 20 Hz
  if (millis() - lastBroadcast >= BROADCAST_MS) {
    broadcastSensorData();
    lastBroadcast = millis();
  }

  delay(1);
}
