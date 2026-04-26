#include "HX711.h"

// ───── pin config ─────
const int HX711_DT  = 18;
const int HX711_SCK = 19;

// ───── calibration constants (from your measurements) ─────
const float CALIB_HEEL = 43801.0;   // counts per kg
const float CALIB_TOE  = 14072.0;   // counts per kg

HX711 scale;

long offset_heel = 0;
long offset_toe  = 0;

long readHeelRaw() {
  scale.set_gain(128);
  scale.read();                  // throwaway
  return scale.read();
}

long readToeRaw() {
  scale.set_gain(32);
  scale.read();                  // throwaway
  return scale.read();
}

void tareBoth(int samples = 10) {
  long sumH = 0, sumT = 0;
  for (int i = 0; i < samples; i++) {
    sumH += readHeelRaw();
    sumT += readToeRaw();
  }
  offset_heel = sumH / samples;
  offset_toe  = sumT / samples;
  Serial.printf("tared: offset_heel=%ld  offset_toe=%ld\n",
                offset_heel, offset_toe);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HX711 calibrated gait sensor ===");

  scale.begin(HX711_DT, HX711_SCK);

  while (!scale.is_ready()) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("\nhx711 ready");

  Serial.println("keep cells unloaded — taring in 2s...");
  delay(2000);
  tareBoth();

  Serial.println("─────────────────────────────────────────\n");
}

void loop() {
  long rawH = readHeelRaw();
  long rawT = readToeRaw();

  float heel_kg = (rawH - offset_heel) / CALIB_HEEL;
  float toe_kg  = (rawT - offset_toe)  / CALIB_TOE;
  float total   = heel_kg - toe_kg;

  Serial.printf("heel: %6.3f kg  |  toe: %6.3f kg  |  total: %6.3f kg\n",
                heel_kg, -toe_kg, total);

  delay(100);
}