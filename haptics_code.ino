#include <Adafruit_NeoPixel.h>

#define MOTOR_PIN 25
#define LED_PIN 5
#define NUM_LEDS 1

Adafruit_NeoPixel pixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// -------- global  variables that get updated  --------
float kneeBendAngle = 0;
float kneeInwardAngle = 0;
float leftLoadCellValue = 0;
float rightLoadCellValue = 0;

// -------- thresholds --------
float warningKneeBendAngle = 80.0;
float dangerousKneeBendAngle = 100.0;

float warningKneeInwardAngle = 8.0;
float dangerousKneeInwardAngle = 12.0;

float warningLoadDiff = 15.0;
float dangerousLoadDiff = 20.0;

// -------- states --------
enum FormState { SAFE, WARNING, DANGEROUS };
FormState state = SAFE;

bool buzzing = false;
unsigned long buzzStart = 0;

// -------- setup --------
void setupSafetyOutputs() {
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  pixel.begin();
  pixel.setBrightness(60);
  setLEDGreen();
}

// -------- in loopL --------
void updateSafetyOutputs() {
  checkForm();
  updateOutputs();
}

// -------- form logic --------
void checkForm() {
  state = SAFE;

  float total = abs(leftLoadCellValue) + abs(rightLoadCellValue);
  float diff = 0;

  if (total > 5) {
    diff = abs(leftLoadCellValue - rightLoadCellValue) / total * 100.0;
  }

  if (kneeBendAngle >= dangerousKneeBendAngle ||
      kneeInwardAngle >= dangerousKneeInwardAngle ||
      diff >= dangerousLoadDiff) {
    state = DANGEROUS;
  }
  else if (kneeBendAngle >= warningKneeBendAngle ||
           kneeInwardAngle >= warningKneeInwardAngle ||
           diff >= warningLoadDiff) {
    state = WARNING;
  }
}

// -------- outputs to haptics  --------
void updateOutputs() {
  if (state == SAFE) {
    setLEDGreen();
    stopMotor();
  }

  else if (state == WARNING) {
    setLEDYellow();
    shortBuzz();
  }

  else if (state == DANGEROUS) {
    setLEDRed();
    longBuzz();
  }
}

// -------- led --------
void setLEDGreen() { pixel.setPixelColor(0, pixel.Color(0,255,0)); pixel.show(); }
void setLEDYellow(){ pixel.setPixelColor(0, pixel.Color(255,150,0)); pixel.show(); }
void setLEDRed()   { pixel.setPixelColor(0, pixel.Color(255,0,0)); pixel.show(); }

// -------- motor --------
void shortBuzz() {
  if (!buzzing) {
    digitalWrite(MOTOR_PIN, HIGH);
    buzzStart = millis();
    buzzing = true;
  }
  if (millis() - buzzStart > 150) {
    stopMotor();
  }
}

void longBuzz() {
  digitalWrite(MOTOR_PIN, HIGH);
}

void stopMotor() {
  digitalWrite(MOTOR_PIN, LOW);
  buzzing = false;
}