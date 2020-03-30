#include <Wire.h>
#include <VL53L1X.h>
#include "ICM20948.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFiManager.h>

// SERVICE and CHARACTERISTICS UUID for BLE
#define TOF_SERVICE_UUID        "efbf52a5-d22b-4808-bccd-b45c5b1d1928"
#define TOF_UUID        "3018bff0-ca31-430b-a6ef-dc5fefd7ee17"

#define MOVEMENT_SERVICE_UUID        "739157ab-dfe6-4dc1-84df-6cd801769d7d"
#define ROTATION_UUID  "2403ca8c-0500-4404-8141-6b0210045365"

//XSHUT OF TOF
#define TOF_1 25
#define TOF_2 26

//TOF Private Variables
VL53L1X sensor1;
unsigned int TOF_cm[3] = {0, 0, 0};
byte TOF_byte[3] = {0, 0, 0}; // 1 byte each data limited to 255cm range
VL53L1X sensor2;
//ICM20948 Private Variables
ICM20948 IMU(Wire, 0x68); // an ICM20948 object with the ICM-20948 sensor on I2C bus 0 with address 0x68
float gyro_float[3] = {0, 0, 0};
float gyro_cal[3] = {0, 0, 0};
float acc_cal[3] = {0, 0, 0} ;
byte rotation_byte[3] = {0, 0, 0};
float elapsedTime, currentTime, previousTime;
int status;
//BLE Private Variables
BLEServer* pServer = NULL;
BLECharacteristic* TOF_Characteristic = NULL;
BLECharacteristic* ROTATION_Characteristic = NULL;
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

struct splitLong { //split long into 2 byte sized packets
  union {
    long value;
    char split[2];
  } __attribute__((packed));
};

struct splitLong LongByteConverter;

void setup()
{
  Wire.begin();
  Serial.begin (115200);
  Sensor_Init();
  BLE_Init();
  SD_Init();
}

void loop()
{
  GetSensor();
  BLE_Notify();
  //  Serial.print("TOF ");
  //  for ( int i = 0 ; i < 2 ; i++) {
  //    Serial.print (i);
  //    Serial.print(": ");
  //    Serial.print(TOF_byte[i]);
  //    Serial.print("\t");
  //  }
  //  Serial.println(" ");
  //  Serial.print("Rotation ");
  //  for ( int i = 0 ; i < 3 ; i++) {
  //    Serial.print (i);
  //    Serial.print(": ");
  //    Serial.print (rotation_byte[i]);
  //    Serial.print("\t");
  //  }
}

void GetSensor() {
  TOF_cm[0] = (sensor1.readRangeContinuousMillimeters()) / 10;
  TOF_cm[1] = (sensor2.readRangeContinuousMillimeters()) / 10;
  for (int i = 0 ; i < 3 ; i++) {
    if ( TOF_cm[i] > 255 ) {
      TOF_byte[i] = 0; // if value greater than 255cm
    }
    else {
      TOF_byte[i] = TOF_cm[i];
    }
  }
  getICM20948();
}
void getICM20948() {
  IMU.readSensor();
  // display the data
  float acc_float[3];
  acc_float[0] = IMU.getAccelX_mss();
  acc_float[1] = IMU.getAccelY_mss();
  acc_float[2] = IMU.getAccelZ_mss();
  for ( int i = 0 ; i < 3  ; i++) {
    acc_float[i] -= acc_cal[i]; // calibration
  }

  // Calculating Roll and Pitch from the accelerometer data
  float accAngleX = (atan(acc_float[1] / sqrt(pow(acc_float[0], 2) + pow(acc_float[2], 2))) * 180 / PI);
  float accAngleY = (atan(-1 * acc_float[0] / sqrt(pow(acc_float[1], 2) + pow(acc_float[2], 2))) * 180 / PI);
  Serial.print("Roll");
  Serial.println (accAngleX);
  Serial.print("Pitch");
  Serial.println(accAngleY);
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000; //seconds elapsed
  float gyro[3];
  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();

  for ( int i = 0 ; i < 3 ; i ++) {
    gyro[i] -= gyro_cal[i]; //calibration
    gyro_float[i] = gyro_float[i] + gyro[i] * elapsedTime;
    gyro_float[i] *= 57.2958; // convert to degrees
  }

  float rotation [3]; // pitch roll yaw
  rotation[2] = rotation[2] + gyro[2] * elapsedTime; //yaw
  // Complementary filter - combine acceleromter and gyro angle values
  rotation[1] = 0.96 * gyro_float[0] + 0.04 * accAngleX; //roll
  rotation[0] = 0.96 * gyro_float[1] + 0.04 * accAngleY; //pitch

  for ( int i = 0 ; i < 3 ; i++) {
    if (rotation[i] < 0) {
      rotation[i] += 90; //angle is 0 to 180
    }
    rotation_byte[i] = uint8_t(rotation[i]);
  }
  //  for ( int i = 0 ; i < 3 ; i++) {
  //    gyro_int[i] = gyro_float[i] * 1000;
  //    if (abs( gyro_int[i] ) > 65535) { // if mod > 65535 set value = 0
  //      gyro_int[i] = 0;
  //    }
  //    else if (gyro_int[i] < 0 ) { //-ve value
  //      gyro_byte [i * 3 ] = 0; //1 ,4 ,7 are sign bytes
  //    }
  //    else {
  //      gyro_byte [i * 3 ] = 1;
  //    }
  //    Serial.print("gyro sign : ");
  //    Serial.println ( gyro_byte[i * 3]);
  //    byte arr[2];
  //    LongByteConverter.value = gyro_int[i];
  //    for ( int j = 1 ; j >= 0 ; j--) {
  //      arr[j] = LongByteConverter.split[j];
  //    }
  //    gyro_byte[i * 3 + 1] = arr [1]; // 1, 4, 7
  //    gyro_byte[i * 3 + 2] = arr[0]; // 2 , 5, 8
  //  }
  //  Serial.print(IMU.getMagX_uT(),6);
  //  Serial.print("\t");
  //  Serial.println(IMU.getTemperature_C(),6);
}

void BLE_Notify() {
  //   notify changed value
  if (deviceConnected) {
    TOF_Characteristic->setValue(TOF_byte, 3); // 1 = 1 byte = 8 bits
    TOF_Characteristic->notify();
    ROTATION_Characteristic->setValue(rotation_byte, 3);
    ROTATION_Characteristic->notify();
    delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    //  }
    delay(600);
  }
}

void LogtoSD(String Data) { //convert data to char array and log into sd card
  char copy[50];
  Data = Data + "\n";
  Data.toCharArray(copy, 50);
  appendFile(SD, "/data.txt", copy);
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
  initICM20948();
}
void initICM20948() {
  while (!Serial) {}
  // start communication with IMU
  status = IMU.begin();
  Serial.print("status = ");
  Serial.println(status);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  //  IMU.configAccel(ICM20948::ACCEL_RANGE_2G, ICM20948::ACCEL_DLPF_BANDWIDTH_50HZ);
  //  IMU.configGyro(ICM20948::GYRO_RANGE_2000DPS, ICM20948::GYRO_DLPF_BANDWIDTH_51HZ);
  //  IMU.setGyroSrd(113); // Output data rate is 1125/(1 + srd) Hz
  //  IMU.setAccelSrd(113);

  for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {//Run this code 2000 times
    IMU.readSensor();
    gyro_cal[0] += IMU.getGyroX_rads();  //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_cal[1] += IMU.getGyroY_rads();
    gyro_cal[2] += IMU.getGyroZ_rads();
    acc_cal[0] += IMU.getAccelX_mss();
    acc_cal[1] += IMU.getAccelX_mss();
    acc_cal[2] += IMU.getAccelX_mss();
    delay(3);    //Delay 3us to simulate the 250Hz program loop
  }
  for (int i = 0 ; i < 3 ; i++) {
    gyro_cal[i] /= 2000;
    acc_cal[i] /= 2000;
  }
  Serial.println("Calibration DONE");
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
  TOF_Characteristic = TOFService->createCharacteristic(
                         TOF_UUID,
                         BLECharacteristic::PROPERTY_READ   |
                         BLECharacteristic::PROPERTY_WRITE  |
                         BLECharacteristic::PROPERTY_NOTIFY |
                         BLECharacteristic::PROPERTY_INDICATE
                       );
  TOF_Characteristic->addDescriptor(new BLE2902());

  // Create a GYROX Characteristic
  ROTATION_Characteristic = MovementService->createCharacteristic(
                              ROTATION_UUID,
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_WRITE  |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_INDICATE
                            );
  ROTATION_Characteristic->addDescriptor(new BLE2902());

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

/* SD Card functions */

void SD_Init() {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  // If the data.txt file doesn't exist, Create a file on the SD card and write the data labels
  File file = SD.open("/data.txt");
  if (!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "Date, Time, TOF_1, TOF_2, Accel, Pitch,Yaw \r\n");
  }
  else {
    Serial.println("File already exists");
  }
  file.close();

}

void readFile(fs::FS & fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS & fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS & fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void deleteFile(fs::FS & fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}
