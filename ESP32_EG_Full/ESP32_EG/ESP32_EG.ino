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

// SERVICE and CHARACTERISTICS UUID for BLE
#define TOF_SERVICE_UUID        "efbf52a5-d22b-4808-bccd-b45c5b1d1928"
#define TOF_UUID        "3018bff0-ca31-430b-a6ef-dc5fefd7ee17"

#define MOVEMENT_SERVICE_UUID        "739157ab-dfe6-4dc1-84df-6cd801769d7d"
#define GYROX_UUID  "2403ca8c-0500-4404-8141-6b0210045365"
#define GYROY_UUID  "90694b00-7941-4734-a674-c893256ed3b4"
#define GYROZ_UUID "a5fe0deb-b6ad-47d0-8e69-b41b9449da07"
#define PITCH_UUID        "75fca86d-0174-4f94-89b7-1b5957d066ae" //up down
#define YAW_UUID        "3f59044f-4fe7-472c-9812-7051f0f35177" // left right

//XSHUT OF TOF
#define TOF_1 25
#define TOF_2 26

//TOF Private Variables
VL53L1X sensor1;
unsigned int TOF_cm[3] = {0, 0, 0};
byte TOF_byte[3] = {0, 0, 0};
VL53L1X sensor2;
//ICM20948 Private Variables
ICM20948 IMU(Wire, 0x68); // an ICM20948 object with the ICM-20948 sensor on I2C bus 0 with address 0x68
float AcX, AcY, AcZ;
float gyroX, gyroY, gyroZ;
byte gyrobyte[3] = {0, 0, 0};
unsigned int gyroX_int, gyroY_int, gyroZ_int;
int status;
int Pitch, Yaw;
//BLE Private Variables
BLEServer* pServer = NULL;
BLECharacteristic* TOF_Characteristic = NULL;
BLECharacteristic* Pitch_Characteristic = NULL;
BLECharacteristic* Yaw_Characteristic = NULL;
BLECharacteristic* GYROX_Characteristic = NULL;
BLECharacteristic* GYROY_Characteristic = NULL;
BLECharacteristic* GYROZ_Characteristic = NULL;
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

struct splitLong { //split long into 4 byte sized packets
  union {
    long value;
    char split[4];
  } __attribute__((packed));
};

struct splitLong LongByteConverter;

void setup()
{
  Wire.begin();
  Serial.begin (115200);
  Sensor_Init();
  BLE_Init(); //TOF_1_Characteristic TOF_2_Characteristic Accel_Characteristic Pitch_Characteristic Yaw_Characteristic
  SD_Init();
}

void loop()
{
  //  GetSensor();
  //  BLE_Notify();
  unsigned long value = 12345678;
  unsigned long val2;
  byte arr[4];

  LongByteConverter.value = value;

  int i ;
  for ( i = 3 ; i >= 0 ; i--) {
    Serial.print(LongByteConverter.split[i], HEX);
    arr[i] = LongByteConverter.split[i];
  }
  Serial.println(" ");
  val2 = * ((unsigned long *) arr ) ;
  Serial.println(val2);
}

void GetSensor() {
  TOF_cm[0] = (sensor1.readRangeContinuousMillimeters()) / 10;
  TOF_cm[1] = (sensor2.readRangeContinuousMillimeters()) / 10;
  int i;
  for (int i = 0 ; i < 3 ; i++) {
    if ( TOF_cm[i] > 255 ) {
      TOF_byte[i] = 0;
    }
    else {
      TOF_byte[i] = TOF_cm[i];
    }
  }
  for ( i = 0 ; i <= 1 ; i ++) {
    Serial.print("TOF_");
    Serial.print(i);
    Serial.println (TOF_byte[i]);
  }
  //  getICM20948();

}
void getICM20948() {
  IMU.readSensor();
  // display the data
  AcX = IMU.getAccelX_mss();
  AcY = IMU.getAccelY_mss();
  AcZ = IMU.getAccelZ_mss();
  //Rotation around Y
  Pitch = atan(-1 * AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI;
  //Rotation around X
  Yaw = atan(-1 * AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI;
  //  Serial.print ("Pitch: ");
  //  Serial.print(Pitch);
  //  Serial.print ("\t");
  //  Serial.print ("Yaw: ");
  //  Serial.print(Yaw);
  //  Serial.println  ("\t");
  Pitch = Pitch + 90; // convert data from -90 < a < 90 to 0 < a < 180
  Yaw = Yaw + 90;
  //  Serial.print ("Pitch: ");
  //  Serial.print(Pitch);
  //  Serial.print ("\t");
  //  Serial.print ("Yaw: ");
  //  Serial.print(Yaw);
  //  Serial.println  ("\t");

  gyroX = IMU.getGyroX_rads();
  gyroY = IMU.getGyroY_rads();
  gyroZ = IMU.getGyroZ_rads();

  gyroX_int = int ( gyroX * 1000); //  cast into 2 bytes == 0 to 65535 and -32768 < a < 32767
  gyroY_int = int (gyroY * 1000); // if value == - 32767, change to 0
  gyroZ_int = int (gyroZ * 1000);// if value == 32768, change to 65535

  gyroX_int = gyroX_int + 32767;
  gyroY_int = gyroX_int + 32767;
  gyroZ_int = gyroX_int + 32767;

  //  Serial.print (" GYROX: ");
  //  Serial.print(gyroX);
  //  Serial.print (":");
  //  Serial.print (gyroX_int);

  //  Serial.print(IMU.getMagX_uT(),6);
  //  Serial.print("\t");
  //  Serial.println(IMU.getTemperature_C(),6);
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
  //  initICM20948();
}
void initICM20948() {
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

void BLE_Notify() {
  //   notify changed value
  unsigned int TOF_NULL = 0;
  if (deviceConnected) {
    TOF_Characteristic->setValue(TOF_byte, 3); // 1 = 1 byte = 8 bits
    TOF_Characteristic->notify();

    //    if ( gyroX_int > 65535) {
    //      GYROX_Characteristic->setValue((uint8_t*)&gyroX_int, 2); // 1 = 1 byte = 8 bits
    //      GYROX_Characteristic->notify();
    //    } else {
    //      GYROX_Characteristic->setValue((uint8_t*)&TOF_NULL, 2);
    //      GYROX_Characteristic->notify();
    //    }
    //    if ( gyroY_int > 65535) {
    //      GYROY_Characteristic->setValue((uint8_t*)&gyroY_int, 2); // 1 = 1 byte = 8 bits
    //      GYROY_Characteristic->notify();
    //    } else {
    //      GYROY_Characteristic->setValue((uint8_t*)&TOF_NULL, 2);
    //      GYROY_Characteristic->notify();
    //    }
    //    if ( gyroZ_int > 65535) {
    //      TOF_1_Characteristic->setValue((uint16_t*)&gyroZ_int, 2); // 1 = 1 byte = 8 bits
    //      TOF_1_Characteristic->notify();
    //    } else {
    //      GYROZ_Characteristic->setValue((uint8_t*)&TOF_NULL, 2);
    //      GYROZ_Characteristic->notify();
    //    }
    Pitch_Characteristic->setValue((uint8_t*)&Pitch, 1);
    Pitch_Characteristic->notify();
    Yaw_Characteristic->setValue((uint8_t*)&Yaw, 1);
    Yaw_Characteristic->notify();

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
  GYROX_Characteristic = MovementService->createCharacteristic(
                           GYROX_UUID,
                           BLECharacteristic::PROPERTY_READ   |
                           BLECharacteristic::PROPERTY_WRITE  |
                           BLECharacteristic::PROPERTY_NOTIFY |
                           BLECharacteristic::PROPERTY_INDICATE
                         );
  GYROX_Characteristic->addDescriptor(new BLE2902());
  // Create a GYROY Characteristic
  GYROY_Characteristic = MovementService->createCharacteristic(
                           GYROY_UUID,
                           BLECharacteristic::PROPERTY_READ   |
                           BLECharacteristic::PROPERTY_WRITE  |
                           BLECharacteristic::PROPERTY_NOTIFY |
                           BLECharacteristic::PROPERTY_INDICATE
                         );
  GYROY_Characteristic->addDescriptor(new BLE2902());
  // Create a GYROZ Characteristic
  GYROZ_Characteristic = MovementService->createCharacteristic(
                           GYROZ_UUID,
                           BLECharacteristic::PROPERTY_READ   |
                           BLECharacteristic::PROPERTY_WRITE  |
                           BLECharacteristic::PROPERTY_NOTIFY |
                           BLECharacteristic::PROPERTY_INDICATE
                         );
  GYROZ_Characteristic->addDescriptor(new BLE2902());

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

/* SD Card functions */
void LogtoSD(String Data) { //convert data to char array and log into sd card
  char copy[50];
  Data = Data + "\n";
  Data.toCharArray(copy, 50);
  appendFile(SD, "/data.txt", copy);
}

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
