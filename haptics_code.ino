i am making a smart acl  brace and this is the ucrrent code. make it sure it is good. it is supposed to have the led be green when the knee angle and load on both feet is in a safe rangel with the display reading "good form" and nothing for the mtoro. when it is entering a warning angle the led is orange and the motor buzzzes a few short buzzes and the display says warning knee is bent getting to be deep or too inward or starting to favor eihter th eleft or right leg and when it reaches teh dangeorus range then led is red and a few big buzzes from the coin motor and the dispaly reads danger knee bent too inward or too deep or you are favoring left or righ t leg tooo much

#include <AdafruitNeoPixel.h>

#define MOTORPIN 25
#define LEDPIN 5
#define NUMLEDS 1

AdafruitNeoPixel pixel(NUMLEDS, LEDPIN, NEOGRB + NEOKHZ800);

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
  pinMode(MOTORPIN, OUTPUT);
  digitalWrite(MOTORPIN, LOW);

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
    digitalWrite(MOTORPIN, HIGH);
    buzzStart = millis();
    buzzing = true;
  }
  if (millis() - buzzStart > 150) {
    stopMotor();
  }
}

void longBuzz() {
  digitalWrite(MOTORPIN, HIGH);
}

void stopMotor() {
  digitalWrite(MOTORPIN, LOW);
  buzzing = false;
}