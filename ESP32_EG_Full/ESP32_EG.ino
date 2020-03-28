#include <Wire.h>
#include <VL53L1X.h>
#include "ICM20948.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// SERVICE and CHARACTERISTICS UUID for BLE
#define TOF_SERVICE_UUID        "efbf52a5-d22b-4808-bccd-b45c5b1d1928"
#define TOF_1_UUID        "3018bff0-ca31-430b-a6ef-dc5fefd7ee17"
#define TOF_2_UUID        "d8286de8-a471-447a-a0af-3e122df5c817"

#define MOVEMENT_SERVICE_UUID        "739157ab-dfe6-4dc1-84df-6cd801769d7d"
#define ACCELEROMETER_UUID        "5c80e46f-5c75-4bd9-bccc-e7f014664620"
#define PITCH_UUID        "75fca86d-0174-4f94-89b7-1b5957d066ae" //up down
#define YAW_UUID        "3f59044f-4fe7-472c-9812-7051f0f35177" // left right

//XSHUT OF TOF
#define TOF_1 25
#define TOF_2 26

//TOF Private Variables
VL53L1X sensor1;
unsigned int TOF_1_cm;
byte TOF_1[8] = [1 , 0 ,0 ,0 ,0 ,0 ,0];
unsigned int TOF_2_cm;
VL53L1X sensor2;
//ICM20948 Private Variables
ICM20948 IMU(Wire, 0x68); // an ICM20948 object with the ICM-20948 sensor on I2C bus 0 with address 0x68
float AcX, AcY, AcZ;
int status;
float angleX, angleY, angleZ;
//BLE Private Variables
BLEServer* pServer = NULL;
BLECharacteristic* TOF_1_Characteristic = NULL;
BLECharacteristic* TOF_2_Characteristic = NULL;
BLECharacteristic* Accel_Characteristic = NULL;
BLECharacteristic* Pitch_Characteristic = NULL;
BLECharacteristic* Yaw_Characteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup()
{
  Wire.begin();
  Serial.begin (115200);
  Sensor_Init();
  BLE_Init(); //TOF_1_Characteristic TOF_2_Characteristic Accel_Characteristic Pitch_Characteristic Yaw_Characteristic
}

void loop()
{
  TOF_1_cm = (sensor1.readRangeContinuousMillimeters()) / 10;
  TOF_2_cm = (sensor2.readRangeContinuousMillimeters()) / 10;
  Serial.print("TOF: [");
  Serial.print(sensor1.readRangeContinuousMillimeters());
  Serial.print(',');
  Serial.print(sensor2.readRangeContinuousMillimeters());
  Serial.print("]");
  Serial.print(TOF_1_cm);
  Serial.print(',');
  Serial.print(TOF_2_cm);
  Serial.print("]");

  IMU.readSensor();
  // display the data
  AcX = IMU.getAccelX_mss();
  AcY = IMU.getAccelY_mss();
  AcZ = IMU.getAccelZ_mss();

  Serial.print(AcX , 2);
  Serial.print(",");
  Serial.print(AcY, 2);
  Serial.print(",");
  Serial.print(AcZ, 2 );
  Serial.println("\t");
  //Rotation around Y
  angleY = atan(-1 * AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI;
  //Rotation Around X
  angleX = atan(-1 * AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI;
  //Rotation Around Z
  angleZ = atan(-1 * AcZ / sqrt(pow(AcX, 2) + pow(AcY, 2))) * 180 / PI;
  Serial.print ("AngleX: ");
  Serial.print(abs(angleZ), 2);
  Serial.print ("\t");
  Serial.print ("AngleY: ");
  Serial.print(abs(angleY), 2);
  Serial.println  ("\t");
  //  Serial.print(IMU.getGyroX_rads(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getGyroY_rads(), 6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getGyroZ_rads(), 6);
  //  Serial.println("\t");
  //  Serial.print(IMU.getMagX_uT(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getMagY_uT(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getMagZ_uT(),6);
  //  Serial.print("\t");
  //  Serial.println(IMU.getTemperature_C(),6);

  // notify changed value
  //  if (deviceConnected) {
  //    TOF_1_Characteristic->setValue((uint8_t*)&value, 4);
  //    pCharacteristic->notify();
  //    value++;
  //    delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  //  }
  //  // disconnecting
  //  if (!deviceConnected && oldDeviceConnected) {
  //    delay(500); // give the bluetooth stack the chance to get things ready
  //    pServer->startAdvertising(); // restart advertising
  //    Serial.println("start advertising");
  //    oldDeviceConnected = deviceConnected;
  //  }
  //  // connecting
  //  if (deviceConnected && !oldDeviceConnected) {
  //    // do stuff here on connecting
  //    oldDeviceConnected = deviceConnected;
  //    //  }
  //    delay(600);
  //  }
}

void Sensor_Init() {
  pinMode(TOF_1, OUTPUT);
  pinMode(TOF_2, OUTPUT);
  digitalWrite(TOF_1, LOW);
  digitalWrite(TOF_2, LOW);
  delay(500);
  pinMode(TOF_1, INPUT);
  delay(150);
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)23); //Set add at 0x23
  pinMode(TOF_2, INPUT);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)24); //Set add at 0x24
  sensor1.setTimeout(500);
  sensor2.setTimeout(500);
  sensor1.startContinuous(33);
  sensor2.startContinuous(33);
  while (!Serial) {}
  // start communication with IMU
  status = IMU.begin();
  Serial.print("status = ");
  Serial.println(status);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
}

void BLE_Init() { //Server --> Service --> Characteristics <-- sensor data input
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

byte TOF_to_Bit(unsigned int reading){ //limited to 0cm to 255cm
  if (reading-128 <0){
    
  }
}
