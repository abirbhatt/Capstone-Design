#include "HX711.h"

// ───── pin config ─────
const int HX711_DT  = 18;   // amp DAT → ESP32 D18
const int HX711_SCK = 19;   // amp CLK → ESP32 D19

HX711 scale;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HX711 dual-channel test ===");
  
  scale.begin(HX711_DT, HX711_SCK);
  
  Serial.print("waiting for hx711");
  int timeout = 0;
  while (!scale.is_ready()) {
    Serial.print(".");
    delay(200);
    timeout++;
    if (timeout > 25) {
      Serial.println("\nHX711 not responding — check wiring:");
      Serial.println("   - VCC and VDD both to 3.3V?");
      Serial.println("   - GND connected?");
      Serial.println("   - DAT → GPIO 18, CLK → GPIO 19?");
      while (1) delay(1000);
    }
  }
  Serial.println("\nhx711 detected");
  Serial.println("press on HEEL cell → channel A should change");
  Serial.println("press on TOE cell  → channel B should change");
  Serial.println("─────────────────────────────────────────\n");
}

void loop() {
  // read channel A (heel) at 128x gain
  scale.set_gain(128);
  scale.read();                  // throwaway after gain switch
  long heel = scale.read();
  
  // read channel B (toe) at 32x gain
  scale.set_gain(32);
  scale.read();                  // throwaway after channel switch
  long toe = scale.read();
  
  Serial.printf("heel (A): %8ld   |   toe (B): %8ld\n", heel, toe);
  
  delay(200);   // ~5Hz, easy to read in serial monitor
}