#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─────────────────────────────────────────────
// SMART ACL BRACE SAFETY OUTPUTS MODULE
// OLED + WS2812B LED + Coin Vibration Motor
// ─────────────────────────────────────────────

// ── PIN CONFIG ───────────────────────────────
#define LED_PIN       5      // WS2812B data pin
#define NUM_LEDS      1
#define MOTOR_PIN     25     // MOSFET gate/base control pin for coin motor

// ── OLED CONFIG ──────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

// ── LED CONFIG ───────────────────────────────
#define BRIGHTNESS    50
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB

CRGB leds[NUM_LEDS];
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── GLOBAL VALUES UPDATED BY MAIN CODE ───────
// Your main code writes to these before calling updateSafetyOutputs()
float kneeBendAngle = 0.0;       // knee flexion angle in degrees
float kneeInwardAngle = 0.0;     // valgus / inward knee angle in degrees

// IMPORTANT:
// For leg favoring, these should be TOTAL leg loads.
// leftLoadCellValue  = injured leg total load
// rightLoadCellValue = healthy leg total load
float leftLoadCellValue = 0.0;
float rightLoadCellValue = 0.0;

// ── KNEE ANGLE THRESHOLDS ────────────────────
// Based on article guidance that ACL restriction is commonly 30–90 degrees.
// We use 75–90 as caution and >90 as danger.
const float WARNING_KNEE_BEND_DEG = 75.0;
const float DANGER_KNEE_BEND_DEG  = 90.0;

// Valgus/inward knee thresholds
// These are engineering alert thresholds and should be tuned during testing.
const float WARNING_VALGUS_DEG = 7.0;
const float DANGER_VALGUS_DEG  = 10.0;

// Load symmetry thresholds
// Uses percent difference between legs.
// Example: injured = 20kg, healthy = 30kg → 33% difference.
const float WARNING_LOAD_DIFF_PCT = 15.0;
const float DANGER_LOAD_DIFF_PCT  = 25.0;

// Ignore symmetry warning if both legs have very little load
const float MIN_TOTAL_LOAD_KG = 1.0;

// ── MOTOR PWM CONFIG ─────────────────────────
const int pwmChannel = 0;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// ── TIMING VARIABLES ─────────────────────────
unsigned long lastOLEDUpdate = 0;
unsigned long lastLongBuzzUpdate = 0;
bool longBuzzState = false;

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────
void setupSafetyOutputs() {
  // LED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  leds[0] = CRGB::Blue;
  FastLED.show();

  // Motor setup
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(MOTOR_PIN, pwmChannel);
  ledcWrite(pwmChannel, 0);

  // OLED setup
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

// ─────────────────────────────────────────────
// LED FUNCTIONS
// ─────────────────────────────────────────────
void setLEDGreen() {
  leds[0] = CRGB::Green;
  FastLED.show();
}

void setLEDYellow() {
  leds[0] = CRGB::Yellow;
  FastLED.show();
}

void setLEDRed() {
  leds[0] = CRGB::Red;
  FastLED.show();
}

// ─────────────────────────────────────────────
// MOTOR FUNCTIONS
// ─────────────────────────────────────────────
void motorOff() {
  ledcWrite(pwmChannel, 0);
  longBuzzState = false;
}

void longBuzzes() {
  // Long repeated buzz pattern for danger
  unsigned long now = millis();

  if (now - lastLongBuzzUpdate >= 800) {
    lastLongBuzzUpdate = now;
    longBuzzState = !longBuzzState;

    if (longBuzzState) {
      ledcWrite(pwmChannel, 230);
    } else {
      ledcWrite(pwmChannel, 0);
    }
  }
}

void shortBuzzes() {
  // Three short buzzes repeating for caution
  unsigned long phase = millis() % 1400;

  if ((phase < 120) ||
      (phase > 250 && phase < 370) ||
      (phase > 500 && phase < 620)) {
    ledcWrite(pwmChannel, 170);
  } else {
    ledcWrite(pwmChannel, 0);
  }

  longBuzzState = false;
}

// ─────────────────────────────────────────────
// LOAD SYMMETRY HELPERS
// ─────────────────────────────────────────────
float getLoadDifferencePercent() {
  float injuredLoad = leftLoadCellValue;
  float healthyLoad = rightLoadCellValue;

  float bigger = max(injuredLoad, healthyLoad);
  float smaller = min(injuredLoad, healthyLoad);

  if (bigger < MIN_TOTAL_LOAD_KG) {
    return 0.0;
  }

  return ((bigger - smaller) / bigger) * 100.0;
}

String getFavoredLegMessage() {
  float injuredLoad = leftLoadCellValue;
  float healthyLoad = rightLoadCellValue;

  // The leg with MORE load is being favored/relied on more.
  if (injuredLoad > healthyLoad) {
    return "Favoring injured";
  } else if (healthyLoad > injuredLoad) {
    return "Favoring healthy";
  } else {
    return "Load balanced";
  }
}

String getShiftMessage() {
  float injuredLoad = leftLoadCellValue;
  float healthyLoad = rightLoadCellValue;

  if (injuredLoad > healthyLoad) {
    return "Shift weight healthy";
  } else if (healthyLoad > injuredLoad) {
    return "Shift weight injured";
  } else {
    return "Keep balance";
  }
}

// ─────────────────────────────────────────────
// OLED DISPLAY
// ─────────────────────────────────────────────
void showOLEDMessage(String line1, String line2, String line3) {
  if (millis() - lastOLEDUpdate < 200) return;
  lastOLEDUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Smart ACL Brace");

  display.setCursor(0, 12);
  display.println("----------------");

  display.setCursor(0, 24);
  display.println(line1);

  display.setCursor(0, 36);
  display.println(line2);

  display.setCursor(0, 48);
  display.println(line3);

  display.display();
}

void showOLEDDataScreen(String status) {
  if (millis() - lastOLEDUpdate < 200) return;
  lastOLEDUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(status);

  display.setCursor(0, 14);
  display.print("Bend: ");
  display.print(kneeBendAngle, 1);
  display.println(" deg");

  display.setCursor(0, 26);
  display.print("Inward: ");
  display.print(kneeInwardAngle, 1);
  display.println(" deg");

  display.setCursor(0, 38);
  display.print("Inj: ");
  display.print(leftLoadCellValue, 1);
  display.print("kg");

  display.setCursor(0, 50);
  display.print("Healthy: ");
  display.print(rightLoadCellValue, 1);
  display.print("kg");

  display.display();
}

// ─────────────────────────────────────────────
// MAIN SAFETY UPDATE
// Called repeatedly from main loop()
// ─────────────────────────────────────────────
void updateSafetyOutputs() {
  float loadDiffPct = getLoadDifferencePercent();

  // DANGER CONDITIONS
  bool kneeTooDeepDanger = kneeBendAngle > DANGER_KNEE_BEND_DEG;
  bool kneeInwardDanger  = kneeInwardAngle > DANGER_VALGUS_DEG;
  bool loadDanger        = loadDiffPct > DANGER_LOAD_DIFF_PCT;

  // WARNING CONDITIONS
  bool kneeTooDeepWarning =
    kneeBendAngle >= WARNING_KNEE_BEND_DEG &&
    kneeBendAngle <= DANGER_KNEE_BEND_DEG;

  bool kneeInwardWarning =
    kneeInwardAngle >= WARNING_VALGUS_DEG &&
    kneeInwardAngle <= DANGER_VALGUS_DEG;

  bool loadWarning =
    loadDiffPct >= WARNING_LOAD_DIFF_PCT &&
    loadDiffPct <= DANGER_LOAD_DIFF_PCT;

  bool danger = kneeTooDeepDanger || kneeInwardDanger || loadDanger;
  bool warning = kneeTooDeepWarning || kneeInwardWarning || loadWarning;

  // ── DANGER: red LED + long buzzes ──────────
  if (danger) {
    setLEDRed();
    longBuzzes();

    if (loadDanger) {
      showOLEDMessage(
        getFavoredLegMessage(),
        "Load is uneven",
        getShiftMessage()
      );
    }
    else if (kneeInwardDanger) {
      showOLEDMessage(
        "Knee too inward",
        "Move knee outward",
        "Fix alignment"
      );
    }
    else if (kneeTooDeepDanger) {
      showOLEDMessage(
        "Knee bent too deep",
        "Reduce knee angle",
        "Stand taller"
      );
    }

    return;
  }

  // ── WARNING: yellow LED + short buzzes ─────
  if (warning) {
    setLEDYellow();
    shortBuzzes();

    if (kneeTooDeepWarning && kneeInwardWarning) {
      showOLEDMessage(
        "CAUTION",
        "Too deep + inward",
        "Adjust form"
      );
    }
    else if (kneeTooDeepWarning) {
      showOLEDMessage(
        "CAUTION",
        "Getting too deep",
        "Reduce bend"
      );
    }
    else if (kneeInwardWarning) {
      showOLEDMessage(
        "CAUTION",
        "Getting too inward",
        "Fix knee line"
      );
    }
    else if (loadWarning) {
      showOLEDMessage(
        "CAUTION",
        getFavoredLegMessage(),
        "Balance weight"
      );
    }

    return;
  }

  // ── SAFE: green LED + motor off ────────────
  setLEDGreen();
  motorOff();
  showOLEDMessage(
    "Form is good",
    "Movement safe",
    "Keep going"
  );
}
