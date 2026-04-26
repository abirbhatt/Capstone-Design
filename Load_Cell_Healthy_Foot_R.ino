#include "HX711.h"

// ───── pin config ─────
const int HX711_DT  = 18;
const int HX711_SCK = 19;

// ───── calibration constants (PAIR B — leg 2) ─────
const float CALIB_HEEL = 40136.0;    // positive factor
const float CALIB_TOE  = -24300.0;    // negative factor (polarity flipped on this cell)

HX711 scale;

long offset_heel = 0;
long offset_toe  = 0;

long readHeelRaw() {
  scale.set_gain(128);
  scale.read();                  // throwaway after gain switch
  return scale.read();
}

long readToeRaw() {
  scale.set_gain(32);
  scale.read();                  // throwaway after channel switch
  return scale.read();
}

void tareBoth(int samples = 20) {
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
  Serial.println("\n=== HX711 gait sensor — PAIR B ===");

  scale.begin(HX711_DT, HX711_SCK);

  while (!scale.is_ready()) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("\nhx711 ready");

  Serial.println("KEEP CELLS UNLOADED — taring in 20s...");
  delay(3000);
  tareBoth();

  Serial.println("─────────────────────────────────────────\n");
}

void loop() {
  // 2-sample average per channel — fast + smooth
  long rawH = (readHeelRaw() + readHeelRaw()) / 2;
  long rawT = (readToeRaw()  + readToeRaw())  / 2;

  float heel_kg = (rawH - offset_heel) / CALIB_HEEL;
  float toe_kg  = (rawT - offset_toe)  / CALIB_TOE;
  float total   = heel_kg + toe_kg;   // PLUS — negative CALIB_TOE handles polarity

  Serial.printf("heel: %6.3f kg  |  toe: %6.3f kg  |  total: %6.3f kg\n",
                heel_kg, toe_kg, total);
}