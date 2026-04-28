#include <WiFi.h>
#include <WebSocketsClient.h>
#include <HX711.h>

// ── WIFI CONFIG ────────────────────────────────
const char* MAIN_AP_SSID = "ACL_Brace";
const char* MAIN_AP_PASS = "aclbrace1";
const char* MAIN_ESP_IP  = "192.168.4.1";
const uint16_t MAIN_WS_PORT = 80;

// ── HX711 PINS ────────────────────────────────
const int HX711_DT  = 18;
const int HX711_SCK = 19;

// ── CALIBRATION (adjust these!) ───────────────
const float CALIB_HEEL = 40136.0f;
const float CALIB_TOE  = -26300.0f;

// ── OBJECTS ───────────────────────────────────
WebSocketsClient wsClient;
HX711 scale;

// ── STATE ─────────────────────────────────────
bool wsConnected = false;

long offset_heel = 0;
long offset_toe  = 0;

float heel_kg = 0.0;
float toe_kg  = 0.0;

// Non-blocking read state machine
enum LCPhase { LC_COMMIT_HEEL, LC_READ_HEEL, LC_COMMIT_TOE, LC_READ_TOE };
LCPhase lcPhase = LC_COMMIT_HEEL;

// ── TARE ──────────────────────────────────────
void tareCells(int samples = 20) {
  Serial.println("[LC] Taring...");

  long sumH = 0, sumT = 0;

  for (int i = 0; i < samples; i++) {
    scale.set_gain(128); scale.read();
    sumH += scale.read();

    scale.set_gain(32); scale.read();
    sumT += scale.read();
  }

  offset_heel = sumH / samples;
  offset_toe  = sumT / samples;

  Serial.println("[LC] Tare complete");
}

// ── INIT LOAD CELLS ───────────────────────────
void initLoadCells() {
  scale.begin(HX711_DT, HX711_SCK);

  Serial.println("[LC] Initializing HX711...");
  while (!scale.is_ready()) {
    Serial.println("[LC] Waiting for HX711...");
    delay(500);
  }

  delay(200);
  tareCells();
}

// ── NON-BLOCKING UPDATE ───────────────────────
void updateLoadCells() {
  if (!scale.is_ready()) return;

  switch (lcPhase) {
    case LC_COMMIT_HEEL:
      scale.set_gain(128);
      scale.read();
      lcPhase = LC_READ_HEEL;
      break;

    case LC_READ_HEEL:
      heel_kg = (scale.read() - offset_heel) / CALIB_HEEL;
      lcPhase = LC_COMMIT_TOE;
      break;

    case LC_COMMIT_TOE:
      scale.set_gain(32);
      scale.read();
      lcPhase = LC_READ_TOE;
      break;

    case LC_READ_TOE:
      toe_kg = (scale.read() - offset_toe) / CALIB_TOE;
      lcPhase = LC_COMMIT_HEEL;
      break;
  }
}

// ── WEBSOCKET EVENTS ──────────────────────────
void onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsConnected = true;
      Serial.println("[WS] Connected");
      break;

    case WStype_DISCONNECTED:
      wsConnected = false;
      Serial.println("[WS] Disconnected");
      break;

    case WStype_TEXT:
      if (strcmp((char*)payload, "tare") == 0) {
        Serial.println("[WS] Tare command received");
        tareCells();
      }
      break;

    default:
      break;
  }
}

// ── SETUP ─────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n-- Load Cell ESP32 Booting --");

  // Load cells
  initLoadCells();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(MAIN_AP_SSID, MAIN_AP_PASS);

  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n[WiFi] Connected");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());

  // WebSocket
  wsClient.begin(MAIN_ESP_IP, MAIN_WS_PORT, "/");
  wsClient.onEvent(onWsEvent);
  wsClient.setReconnectInterval(2000);

  Serial.println("[WS] Connecting to main ESP32...");
}

// ── LOOP ──────────────────────────────────────
uint32_t lastSend = 0;
const uint32_t SEND_INTERVAL = 50; // 20 Hz

void loop() {
  wsClient.loop();

  updateLoadCells();

  if (wsConnected && millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();

    float total = heel_kg + toe_kg;

    char msg[160];
    snprintf(msg, sizeof(msg),
      "{"
      "\"healthy\":1,"
      "\"knee_angle\":0,"
      "\"valgus\":0,"
      "\"heel_kg\":%.2f,"
      "\"toe_kg\":%.2f,"
      "\"load_total\":%.2f,"
      "\"ts\":%lu"
      "}",
      heel_kg, toe_kg, total, millis()
    );

    wsClient.sendTXT(msg);
  }

  delay(1);
}