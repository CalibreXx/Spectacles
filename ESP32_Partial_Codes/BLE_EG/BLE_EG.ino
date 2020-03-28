/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updated by chegewara

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
   And has a characteristic of: beb5483e-36e1-4688-b7f5-ea07361b26a8

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   A connect hander associated with the server starts a background task that performs notification
   every couple of seconds.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic* TOF_1_Characteristic = NULL;
BLECharacteristic* TOF_2_Characteristic = NULL;
BLECharacteristic* Accel_Characteristic = NULL;
BLECharacteristic* Pitch_Characteristic = NULL;
BLECharacteristic* Yaw_Characteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define TOF_SERVICE_UUID        "efbf52a5-d22b-4808-bccd-b45c5b1d1928"
#define TOF_1_UUID        "3018bff0-ca31-430b-a6ef-dc5fefd7ee17"
#define TOF_2_UUID        "d8286de8-a471-447a-a0af-3e122df5c817"

#define MOVEMENT_SERVICE_UUID        "739157ab-dfe6-4dc1-84df-6cd801769d7d"
#define ACCELEROMETER_UUID        "5c80e46f-5c75-4bd9-bccc-e7f014664620"
#define PITCH_UUID        "75fca86d-0174-4f94-89b7-1b5957d066ae" //up down
#define YAW_UUID        "3f59044f-4fe7-472c-9812-7051f0f35177" // left right

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};



void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32-BRYAN");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *TOFService = pServer->createService(TOF_SERVICE_UUID);
  BLEService *MovementService = pServer->createService(MOVEMENT_SERVICE_UUID);

  // Create a BLE TOF_1 Characteristic
  TOF_1_Characteristic = TOFService->createCharacteristic(
                      TOF_1_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  TOF_1_Characteristic->addDescriptor(new BLE2902());
  
  // Create a BLE TOF_2 Characteristic
  TOF_2_Characteristic = TOFService->createCharacteristic(
                      TOF_2_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  TOF_2_Characteristic->addDescriptor(new BLE2902());
  
  // Create a BLE ACCELL Characteristic
  Accel_Characteristic = MovementService->createCharacteristic(
                      ACCELEROMETER_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  Accel_Characteristic->addDescriptor(new BLE2902());
  
  // Create a BLE PITCH Characteristic
  Pitch_Characteristic = MovementService->createCharacteristic(
                      PITCH_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  Pitch_Characteristic->addDescriptor(new BLE2902());
  
  // Create a BLE YAW Characteristic
  Yaw_Characteristic = MovementService->createCharacteristic(
                      YAW_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  Yaw_Characteristic->addDescriptor(new BLE2902());
  

  // Start the service
  TOFService->start();
  MovementService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(TOF_SERVICE_UUID);
  pAdvertising->addServiceUUID(MOVEMENT_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

}
