#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

// ---------- OUTPUT PINS ----------
#define MOTOR_LEFT_PIN 25
#define MOTOR_RIGHT_PIN 26
#define LED_PIN 5
#define NUM_LEDS 1

// ---------- OLED ----------
extern Adafruit_SSD1306 display;

// ---------- LED ----------
Adafruit_NeoPixel pixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// =====================================================
// glob sensor variables
//these get update with code
// =====================================================
float kneeBendAngle = 0.0;        // degrees
float kneeInwardAngle = 0.0;      // degrees
float leftLoadCellValue = 0.0;
float rightLoadCellValue = 0.0;

// THRESHOLDS
// warning around 80 deg, danger around 100 deg
// =====================================================
float warningKneeBendAngle = 80.0;
float dangerousKneeBendAngle = 100.0;

float warningKneeInwardAngle = 8.0;
float dangerousKneeInwardAngle = 12.0;

float warningLoadDifferencePercent = 15.0;
float dangerousLoadDifferencePercent = 20.0;

// =====================================================
// status variables
// =====================================================
enum FormState {
  SAFE,
  WARNING,
  DANGEROUS
};

FormState formState = SAFE;

String formStatus = "Good form";
String formReason = "";

// =====================================================
// timing 
// =====================================================
unsigned long lastOLEDUpdate = 0;
const unsigned long oledUpdateInterval = 200;

bool warningBuzzDone = false;
unsigned long buzzStartTime = 0;
bool buzzing = false;

const unsigned long warningBuzzDuration = 150;
const unsigned long dangerousBuzzDuration = 800;

// =====================================================
//call this once in setup
// =====================================================
void setupSafetyOutputs() {
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);

  digitalWrite(MOTOR_LEFT_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_PIN, LOW);

  pixel.begin();
  pixel.setBrightness(60);
  setLEDGreen();
}

// =====================================================
// call this repeatedly in loop
// =====================================================
void updateSafetyOutputs() {
  checkFormStatus();
  updateLEDAndMotors();

  if (millis() - lastOLEDUpdate >= oledUpdateInterval) {
    lastOLEDUpdate = millis();
    updateSafetyOLED();
  }
}

// =====================================================
// form logic 
// =====================================================
void checkFormStatus() {
  FormState previousState = formState;

  formState = SAFE;
  formStatus = "Good form";
  formReason = "";

  float totalLoad = abs(leftLoadCellValue) + abs(rightLoadCellValue);
  float loadDifferencePercent = 0.0;

  if (totalLoad > 5) {
    loadDifferencePercent =
      abs(leftLoadCellValue - rightLoadCellValue) / totalLoad * 100.0;
  }

  // ---------- DANGEROUS CHECKS ----------
  if (kneeBendAngle >= dangerousKneeBendAngle) {
    formState = DANGEROUS;
    formStatus = "DANGEROUS";
    formReason = "Knee bend too deep";
  }
  else if (kneeInwardAngle >= dangerousKneeInwardAngle) {
    formState = DANGEROUS;
    formStatus = "DANGEROUS";
    formReason = "Inward knee bend";
  }
  else if (loadDifferencePercent >= dangerousLoadDifferencePercent) {
    formState = DANGEROUS;
    formStatus = "DANGEROUS";
    formReason = "Load difference";
  }

  // ---------- WARNING CHECKS ----------
  else if (kneeBendAngle >= warningKneeBendAngle) {
    formState = WARNING;
    formStatus = "WARNING";
    formReason = "Knee bend getting deep";
  }
  else if (kneeInwardAngle >= warningKneeInwardAngle) {
    formState = WARNING;
    formStatus = "WARNING";
    formReason = "Knee bending inward";
  }
  else if (loadDifferencePercent >= warningLoadDifferencePercent) {
    formState = WARNING;
    formStatus = "WARNING";
    formReason = "One leg favored";
  }

  // Reset one-time warning buzz when state changes
  if (formState != previousState) {
    warningBuzzDone = false;
    stopBuzz();
  }
}

// =====================================================
// led and motor logic 
// =====================================================
void updateLEDAndMotors() {
  if (formState == SAFE) {
    setLEDGreen();
    stopBuzz();
  }

  else if (formState == WARNING) {
    setLEDYellow();

    if (!warningBuzzDone && !buzzing) {
      startBuzz(warningBuzzDuration);
      warningBuzzDone = true;
    }

    updateBuzzTimer();
  }

  else if (formState == DANGEROUS) {
    setLEDRed();

    if (!buzzing) {
      startBuzz(dangerousBuzzDuration);
    }

    updateBuzzTimer();
  }
}

// =====================================================
// display logic 
// =====================================================
void updateSafetyOLED() {
  display.clearDisplay();
  display.setTextColor(WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ACL Brace Monitor");

  display.setCursor(0, 16);

  if (formState == SAFE) {
    display.println("GOOD FORM");
  } 
  else {
    display.println(formStatus);
    display.setCursor(0, 28);
    display.println(formReason);
  }

  display.setCursor(0, 44);
  display.print("Bend: ");
  display.print(kneeBendAngle, 1);
  display.println(" deg");

  display.setCursor(0, 56);
  display.print("In: ");
  display.print(kneeInwardAngle, 1);
  display.print(" L:");
  display.print(leftLoadCellValue, 0);
  display.print(" R:");
  display.print(rightLoadCellValue, 0);

  display.display();
}

// =====================================================
// led colors 
// =====================================================
void setLEDGreen() {
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();
}

void setLEDYellow() {
  pixel.setPixelColor(0, pixel.Color(255, 150, 0));
  pixel.show();
}

void setLEDRed() {
  pixel.setPixelColor(0, pixel.Color(255, 0, 0));
  pixel.show();
}

// =====================================================
// motor buzzes 
// =====================================================
void startBuzz(unsigned long duration) {
  buzzStartTime = millis();
  buzzing = true;

  digitalWrite(MOTOR_LEFT_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_PIN, HIGH);
}

void updateBuzzTimer() {
  unsigned long duration;

  if (formState == WARNING) {
    duration = warningBuzzDuration;
  } else {
    duration = dangerousBuzzDuration;
  }

  if (buzzing && millis() - buzzStartTime >= duration) {
    stopBuzz();
  }
}

void stopBuzz() {
  buzzing = false;

  digitalWrite(MOTOR_LEFT_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_PIN, LOW);
}