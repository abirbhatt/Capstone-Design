#include "HX711.h"

// ───── pick which leg this ESP is for ─────
#define IS_INJURED_LEG  true   // set to false on the healthy leg's ESP

// ───── pin config ─────
const int HX711_DT  = 18;
const int HX711_SCK = 19;

// ───── calibration constants ─────
#if IS_INJURED_LEG
  const float CALIB_HEEL = 43801.0;
  const float CALIB_TOE  = 14072.0;
  const char* LEG_NAME   = "injured";
#else
  const float CALIB_HEEL = 40136.0;
  const float CALIB_TOE  = -24300.0;
  const char* LEG_NAME   = "healthy";
#endif

// ───── load cell state (globals) ─────
HX711 scale;
long offset_heel = 0;
long offset_toe  = 0;
float heel_kg = 0, toe_kg = 0;
bool cells_ready = false;

// ───── load cell helpers ─────
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

void tareCells(int samples = 20) {
  long sumH = 0, sumT = 0;
  for (int i = 0; i < samples; i++) {
    sumH += readHeelRaw();
    sumT += readToeRaw();
  }
  offset_heel = sumH / samples;
  offset_toe  = sumT / samples;
}

void initLoadCells() {
  scale.begin(HX711_DT, HX711_SCK);
  
  unsigned long start = millis();
  while (!scale.is_ready() && millis() - start < 3000) {
    delay(50);
  }
  
  if (scale.is_ready()) {
    delay(500);
    tareCells();
    cells_ready = true;
    Serial.println("load cells: ready");
  } else {
    Serial.println("load cells: NOT FOUND");
  }
}

void updateLoadCells() {
  if (!cells_ready) return;
  
  long rawH = (readHeelRaw() + readHeelRaw()) / 2;
  long rawT = (readToeRaw()  + readToeRaw())  / 2;
  
  heel_kg = (rawH - offset_heel) / CALIB_HEEL;
  toe_kg  = (rawT - offset_toe)  / CALIB_TOE;
}

float getHeel()  { return heel_kg; }
float getToe()   { return toe_kg; }
float getTotal() { return heel_kg + toe_kg; }

// ───── setup / loop (DELETE these if merging into existing sketch) ─────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.print("=== leg: "); Serial.print(LEG_NAME); Serial.println(" ===");
  
  initLoadCells();
  
  // your friend's other init code goes here:
  // - wifi connect
  // - websocket setup  
  // - IMU init
  // - buzzer pin setup
}

void loop() {
  updateLoadCells();
  
  // your friend's loop code goes here:
  // - read IMU
  // - check buzzer threshold
  // - build JSON payload using getHeel(), getToe(), getTotal()
  // - send via websocket
  
  // example payload:
  // String payload = "{\"leg\":\"" + String(LEG_NAME) + "\","
  //                  "\"heel\":" + String(getHeel(), 2) + ","
  //                  "\"toe\":"  + String(getToe(), 2)  + ","
  //                  "\"total\":"+ String(getTotal(),2) + "}";
  // ws.textAll(payload);
  
  delay(50);
}
