#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs - you can keep these or generate your own at uuidgenerator.net
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcd1234-ab12-ab12-ab12-abcdef123456"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float kneeAngle = 0.0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    BLEDevice::startAdvertising(); // restart advertising after disconnect
  }
};

void setup() {
  Serial.begin(115200);

  BLEDevice::init("SmartACLBrace");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902()); // enables notifications

  pService->start();
  BLEDevice::startAdvertising();
  Serial.println("BLE broadcasting...");
}

void loop() {
  if (deviceConnected) {
    // Simulate a knee angle sweeping 0–90 degrees
    kneeAngle = (kneeAngle >= 90.0) ? 0.0 : kneeAngle + 1.0;

    char buf[16];
    dtostrf(kneeAngle, 4, 1, buf);
    pCharacteristic->setValue(buf);
    pCharacteristic->notify();

    Serial.print("Sent: ");
    Serial.println(buf);
  }
  delay(100);
}