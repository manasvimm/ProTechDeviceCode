#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BLE2902.h>

// Create BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055();

// BLE Variables
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef01-1234-5678-1234-56789abcdef0"

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize BNO055 Sensor
  if (!bno.begin()) {
    while (1);
  }

  // Initialize BLE
  BLEDevice::init("ESP32-BNO055");  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  // Start Advertising BLE
  BLEDevice::getAdvertising()->start();
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  String sensorData = String(event.acceleration.x) + "," +
                      String(event.acceleration.y) + "," +
                      String(event.acceleration.z);

  if (deviceConnected) {
    pCharacteristic->setValue(sensorData.c_str());
    pCharacteristic->notify();
  }

  delay(100);  // Adjust delay for faster logging
}
