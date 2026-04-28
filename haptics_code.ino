#include <Adafruit_NeoPixel.h>

#define MOTOR_PIN 25
#define LED_PIN 5
#define NUM_LEDS 1

Adafruit_NeoPixel pixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// -------- sensor values (update these from your sensors) --------
float kneeBendAngle = 0;
float kneeInwardAngle = 0;
float leftLoadCellValue = 0;
float rightLoadCellValue = 0;

// -------- thresholds --------
const float warningKneeBendAngle = 80.0;
const float dangerousKneeBendAngle = 100.0;

const float warningKneeInwardAngle = 8.0;
const float dangerousKneeInwardAngle = 12.0;

const float warningLoadDiff = 15.0;
const float dangerousLoadDiff = 20.0;

// -------- states --------
enum FormState { SAFE, WARNING, DANGEROUS };
FormState currentState = SAFE;
FormState previousState = SAFE;

// -------- buzz pattern control --------
bool buzzing = false;
unsigned long buzzTimer = 0;
int buzzCount = 0;
int buzzPhase = 0;  // 0 = motor on, 1 = motor off (gap)

// Buzz parameters
const int SHORT_BUZZ_ON = 100;
const int SHORT_BUZZ_OFF = 100;
const int SHORT_BUZZ_COUNT = 3;

const int LONG_BUZZ_ON = 300;
const int LONG_BUZZ_OFF = 150;
const int LONG_BUZZ_COUNT = 3;

// -------- function declarations --------
void setupSafetyOutputs();
void updateSafetyOutputs();
void checkForm();
void updateOutputs();
void setLEDGreen();
void setLEDOrange();
void setLEDRed();
void startBuzzPattern(int onTime, int offTime, int count);
void updateBuzzPattern();
void stopMotor();
void updateDisplay(const char* line1, const char* line2);
String getWarningReason();
String getDangerReason();

// -------- setup --------
void setupSafetyOutputs() {
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  pixel.begin();
  pixel.setBrightness(60);
  setLEDGreen();
  updateDisplay("Good Form", "");
}

// -------- call this in loop() --------
void updateSafetyOutputs() {
  checkForm();
  updateOutputs();
  updateBuzzPattern();
}

// -------- form logic --------
void checkForm() {
  previousState = currentState;
  currentState = SAFE;

  float total = abs(leftLoadCellValue) + abs(rightLoadCellValue);
  float diff = 0;

  if (total > 5) {
    diff = abs(leftLoadCellValue - rightLoadCellValue) / total * 100.0;
  }

  if (kneeBendAngle >= dangerousKneeBendAngle ||
      kneeInwardAngle >= dangerousKneeInwardAngle ||
      diff >= dangerousLoadDiff) {
    currentState = DANGEROUS;
  }
  else if (kneeBendAngle >= warningKneeBendAngle ||
           kneeInwardAngle >= warningKneeInwardAngle ||
           diff >= warningLoadDiff) {
    currentState = WARNING;
  }
}

// -------- update outputs based on state --------
void updateOutputs() {
  bool stateChanged = (currentState != previousState);

  if (currentState == SAFE) {
    setLEDGreen();
    stopMotor();
    if (stateChanged) {
      updateDisplay("Good Form", "");
    }
  }
  else if (currentState == WARNING) {
    setLEDOrange();
    if (stateChanged) {
      startBuzzPattern(SHORT_BUZZ_ON, SHORT_BUZZ_OFF, SHORT_BUZZ_COUNT);
      String reason = getWarningReason();
      updateDisplay("WARNING", reason.c_str());
    }
  }
  else if (currentState == DANGEROUS) {
    setLEDRed();
    if (stateChanged) {
      startBuzzPattern(LONG_BUZZ_ON, LONG_BUZZ_OFF, LONG_BUZZ_COUNT);
      String reason = getDangerReason();
      updateDisplay("DANGER", reason.c_str());
    }
  }
}

// -------- reason strings --------
String getWarningReason() {
  float total = abs(leftLoadCellValue) + abs(rightLoadCellValue);
  float diff = (total > 5) ? abs(leftLoadCellValue - rightLoadCellValue) / total * 100.0 : 0;

  if (kneeBendAngle >= warningKneeBendAngle) return "Knee bending deep";
  if (kneeInwardAngle >= warningKneeInwardAngle) return "Knee moving inward";
  if (diff >= warningLoadDiff) {
    return (leftLoadCellValue > rightLoadCellValue) ? "Favoring left leg" : "Favoring right leg";
  }
  return "";
}

String getDangerReason() {
  float total = abs(leftLoadCellValue) + abs(rightLoadCellValue);
  float diff = (total > 5) ? abs(leftLoadCellValue - rightLoadCellValue) / total * 100.0 : 0;

  if (kneeBendAngle >= dangerousKneeBendAngle) return "Knee bent too deep!";
  if (kneeInwardAngle >= dangerousKneeInwardAngle) return "Knee too far inward!";
  if (diff >= dangerousLoadDiff) {
    return (leftLoadCellValue > rightLoadCellValue) ? "Favoring left too much!" : "Favoring right too much!";
  }
  return "";
}

// -------- LED functions --------
void setLEDGreen()  { pixel.setPixelColor(0, pixel.Color(0, 255, 0));   pixel.show(); }
void setLEDOrange() { pixel.setPixelColor(0, pixel.Color(255, 100, 0)); pixel.show(); }
void setLEDRed()    { pixel.setPixelColor(0, pixel.Color(255, 0, 0));   pixel.show(); }

// -------- buzz pattern control --------
int buzzOnTime = 0;
int buzzOffTime = 0;
int buzzTarget = 0;

void startBuzzPattern(int onTime, int offTime, int count) {
  buzzOnTime = onTime;
  buzzOffTime = offTime;
  buzzTarget = count;
  buzzCount = 0;
  buzzPhase = 0;
  buzzing = true;
  digitalWrite(MOTOR_PIN, HIGH);
  buzzTimer = millis();
}

void updateBuzzPattern() {
  if (!buzzing) return;

  unsigned long elapsed = millis() - buzzTimer;

  if (buzzPhase == 0) {  // Motor ON phase
    if (elapsed >= buzzOnTime) {
      digitalWrite(MOTOR_PIN, LOW);
      buzzPhase = 1;
      buzzTimer = millis();
      buzzCount++;
    }
  }
  else {  // Motor OFF phase (gap)
    if (elapsed >= buzzOffTime) {
      if (buzzCount >= buzzTarget) {
        buzzing = false;
        stopMotor();
      } else {
        digitalWrite(MOTOR_PIN, HIGH);
        buzzPhase = 0;
        buzzTimer = millis();
      }
    }
  }
}

void stopMotor() {
  digitalWrite(MOTOR_PIN, LOW);
  buzzing = false;
}

// -------- display placeholder --------
// Replace this with your actual display code (OLED, LCD, etc.)
void updateDisplay(const char* line1, const char* line2) {
  // Example for Serial (replace with your display library):
  Serial.println(line1);
  if (strlen(line2) > 0) {
    Serial.println(line2);
  }
  
  // For an OLED like SSD1306:
  // display.clearDisplay();
  // display.setCursor(0, 0);
  // display.println(line1);
  // display.println(line2);
  // display.display();
}
