#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── PIN CONFIG ───────────────────────────────
#define LED_PIN       5
#define NUM_LEDS      1
#define MOTOR_PIN     25

// OLED
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

// ── THRESHOLDS ───────────────────────────────
// Danger thresholds
const float DANGER_KNEE_BEND_DEG = 90.0;
const float DANGER_VALGUS_DEG    = 10.0;
const float DANGER_LOAD_DIFF_KG  = 8.0;

// Warning thresholds
const float WARNING_KNEE_BEND_DEG = 75.0;
const float WARNING_VALGUS_DEG    = 7.0;
const float WARNING_LOAD_DIFF_KG  = 5.0;

// ── LED CONFIG ───────────────────────────────
#define BRIGHTNESS 50
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// These are updated by your main code before updateSafetyOutputs()
float kneeBendAngle = 0.0;
float kneeInwardAngle = 0.0;
float leftLoadCellValue = 0.0;    // injured leg / heel in your current main code
float rightLoadCellValue = 0.0;   // healthy leg or comparison value if you update it

// PWM motor setup
const int pwmChannel = 0;
const int pwmFreq = 5000;
const int pwmResolution = 8;

unsigned long lastOLEDUpdate = 0;
unsigned long lastBuzzUpdate = 0;
bool buzzState = false;

// ── SETUP ────────────────────────────────────
void setupSafetyOutputs() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  leds[0] = CRGB::Blue;
  FastLED.show();

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(MOTOR_PIN, pwmChannel);
  ledcWrite(pwmChannel, 0);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found at 0x3C");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Smart ACL Brace");
    display.println("Outputs Ready");
    display.display();
  }

  Serial.println("Safety outputs ready");
}

// ── LED ──────────────────────────────────────
void setLED(bool danger, bool warning) {
  if (danger) {
    leds[0] = CRGB::Red;
  } else if (warning) {
    leds[0] = CRGB::Yellow;
  } else {
    leds[0] = CRGB::Green;
  }
  FastLED.show();
}

// ── HAPTICS ──────────────────────────────────
void motorOff() {
  ledcWrite(pwmChannel, 0);
  buzzState = false;
}

void longBuzzPattern() {
  // Long repeated buzzes for danger
  unsigned long now = millis();

  if (now - lastBuzzUpdate >= 700) {
    lastBuzzUpdate = now;
    buzzState = !buzzState;
    ledcWrite(pwmChannel, buzzState ? 230 : 0);
  }
}

void shortBuzzPattern() {
  // A few short buzzes for caution
  unsigned long now = millis();
  unsigned long phase = now % 1200;

  if (phase < 120 || (phase > 250 && phase < 370) || (phase > 500 && phase < 620)) {
    ledcWrite(pwmChannel, 170);
  } else {
    ledcWrite(pwmChannel, 0);
  }
}

// ── OLED ─────────────────────────────────────
void showOLEDMessage(String line1, String line2, String line3) {
  if (millis() - lastOLEDUpdate < 200) return;
  lastOLEDUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.println("Smart ACL Brace");
  display.println("----------------");

  display.setCursor(0, 18);
  display.println(line1);

  display.setCursor(0, 30);
  display.println(line2);

  display.setCursor(0, 42);
  display.println(line3);

  display.setCursor(0, 54);
  display.print("B:");
  display.print(kneeBendAngle, 0);
  display.print(" V:");
  display.print(kneeInwardAngle, 0);
  display.print(" L:");
  display.print(leftLoadCellValue, 1);
  display.print(" R:");
  display.print(rightLoadCellValue, 1);

  display.display();
}

// ── MAIN UPDATE ──────────────────────────────
void updateSafetyOutputs() {
  float loadDifference = abs(leftLoadCellValue - rightLoadCellValue);

  bool kneeTooDeepDanger = kneeBendAngle >= DANGER_KNEE_BEND_DEG;
  bool kneeInwardDanger  = kneeInwardAngle >= DANGER_VALGUS_DEG;
  bool loadDanger        = loadDifference >= DANGER_LOAD_DIFF_KG;

  bool kneeTooDeepWarning = kneeBendAngle >= WARNING_KNEE_BEND_DEG && kneeBendAngle < DANGER_KNEE_BEND_DEG;
  bool kneeInwardWarning  = kneeInwardAngle >= WARNING_VALGUS_DEG && kneeInwardAngle < DANGER_VALGUS_DEG;
  bool loadWarning        = loadDifference >= WARNING_LOAD_DIFF_KG && loadDifference < DANGER_LOAD_DIFF_KG;

  bool danger = kneeTooDeepDanger || kneeInwardDanger || loadDanger;
  bool warning = kneeTooDeepWarning || kneeInwardWarning || loadWarning;

  setLED(danger, warning);

  if (danger) {
    longBuzzPattern();
  } else if (warning) {
    shortBuzzPattern();
  } else {
    motorOff();
  }

  // Priority 1: load imbalance / favoring one leg
  if (loadDanger) {
    if (leftLoadCellValue < rightLoadCellValue) {
      showOLEDMessage("Favoring RIGHT leg", "Load is uneven", "Shift weight left");
    } else {
      showOLEDMessage("Favoring LEFT leg", "Load is uneven", "Shift weight right");
    }
  }

  // Priority 2: knee too inward
  else if (kneeInwardDanger) {
    showOLEDMessage("Knee too inward", "Correct alignment", "Move knee outward");
  }

  // Priority 3: knee bent too deep
  else if (kneeTooDeepDanger) {
    showOLEDMessage("Knee bent too deep", "Reduce knee angle", "Stand taller");
  }

  // Warnings
  else if (kneeTooDeepWarning && kneeInwardWarning) {
    showOLEDMessage("CAUTION", "Too deep + inward", "Adjust form");
  } else if (kneeTooDeepWarning) {
    showOLEDMessage("CAUTION", "Getting too deep", "Slow down");
  } else if (kneeInwardWarning) {
    showOLEDMessage("CAUTION", "Getting too inward", "Correct knee");
  } else if (loadWarning) {
    showOLEDMessage("CAUTION", "Uneven loading", "Balance weight");
  }

  // Everything okay
  else {
    showOLEDMessage("Form is good", "Movement safe", "Keep going");
  }
}
