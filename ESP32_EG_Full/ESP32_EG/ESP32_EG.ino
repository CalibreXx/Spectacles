#include <Wire.h>
#include <VL53L1X.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <FreeSixIMU.h>
#include "RTClib.h"

// SERVICE and CHARACTERISTICS UUID for BLE
#define TOF_SERVICE_UUID        "efbf52a5-d22b-4808-bccd-b45c5b1d1928"
#define TOF_UUID        "3018bff0-ca31-430b-a6ef-dc5fefd7ee17"
#define LDR_UUID "e9ff40d9-21da-44dd-b125-ad2d8ef6b026"

#define MOVEMENT_SERVICE_UUID        "739157ab-dfe6-4dc1-84df-6cd801769d7d"
#define ROTATION_UUID  "2403ca8c-0500-4404-8141-6b0210045365"
#define ACCEL_UUID "d0b5f187-ac23-459f-b44b-e20d50bcf656"

#define TIME_SERVICE_UUID "57675859-a6f4-4445-9492-051aa8514552"
#define TIME_UUID "10ccece5-e44b-4502-8b69-09646d4072e1" // Write Date time etc to this uuid to update time on ESP32

#define DATA_SERVICE_UUID "b8ec9f13-81e2-489f-b736-f4e440c86e03"
#define DATA_CALL_UUID "5022e570-0f19-4357-848a-fc74234b1348" // Write to this uuid to req for data xfer
#define DATA_SEND_UUID "38ca7184-8eeb-481f-9197-2c106f076031"

//XSHUT OF TOF
#define TOF_1 25 //0x23 LEFT
#define TOF_2 26 //0x24 CENTRE
#define TOF_3 27 //0x25 RIGHT

//TOF Private Variables
VL53L1X sensor1; VL53L1X sensor2; VL53L1X sensor3;
unsigned int TOF_cm[3] = {0, 0, 0};
byte TOF_byte[3] = {0, 0, 0}; // 1 byte each sensor limited to 255cm range

// LDR
const int LDR_PIN = 34; // analog pins
int lightVal;
byte light_byte[2] = { 0 , 0 }; // 2 bytes range from 0 to 65,535

//ICM20948 Private Variables
float IMU_cal[3] = {0, 0, 0};
byte rotation_byte[3] = {0, 0, 0}; // Yaw Pitch Roll
byte acceleration[1] = {0};

//BLE Private Variables
BLEServer* pServer = NULL;
BLECharacteristic* TOF_Characteristic = NULL;
BLECharacteristic* ROTATION_Characteristic = NULL;
BLECharacteristic* ACCEL_Characteristic = NULL;
BLECharacteristic* LDR_Characteristic = NULL;
BLECharacteristic* TIME_Characteristic = NULL;
BLECharacteristic* DATA_CALL_Characteristic = NULL;
BLECharacteristic* DATA_SEND_Characteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

//RTC
long unsigned int epoch;

const unsigned long loopInterval = 1000;
unsigned long previousTime = 0 ;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class TimeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *TIME_Characteristic) {
      std::string value = TIME_Characteristic->getValue();
      int BLETime[15]; //day month hour, min, sec , yyyy dayof the week
      int variable[6]; //day month hour, min, sec , yyyy dayof the week
      int j = 0;
      if (value.length() == 14 ) {
        for ( int i = 0 ; i < value.length() ; i++) {
          if ( value[i] >= '0' && value[i] <= '9') {
            BLETime[j] = value[i] - '0';
            j++;
          }
        }
        for ( int i = 0 ; i < 5; i++) { //DD MM HH SS MM
          variable[i] = BLETime[i * 2] * 10 + BLETime[i * 2 + 1];
        }
        variable[5] = BLETime[10] * 1000 + BLETime[11] * 100 + BLETime[12] * 10 + BLETime[13];
        RTC_DS3231 rtc;
        rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
        for ( int i = 0 ; i < 6 ; i++) {
          Serial.println(variable[i]);
        }
        Serial.print("Set new time completed");
      }
    }
};

class DataCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *DATA_CALL_Characteristic) {
      std::string value = TIME_Characteristic->getValue();
      if (value == "1" ) {
        //send massive load of data
      }
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
  RTC_DS3231 rtc;
  Wire.begin();
  Serial.begin (115200);
  Sensor_Init();
  BLE_Init();
  SD_Init();
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
}

void loop()
{
  RTC_DS3231 rtc;
  DateTime now = rtc.now();
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= loopInterval) {
    epoch = now.unixtime();
    GetSensor();
    BLE_Notify();
    Serial.print("Epoch: "); Serial.println(epoch);
    Serial.print("TOF ");
    for ( int i = 0 ; i < 3 ; i++) {
      Serial.print (i); Serial.print(": ");
      Serial.print(TOF_byte[i]); Serial.print("\t");
    }
    Serial.print ( "Acceleration: "); Serial.println(acceleration[0]);
    Serial.print("Rotation ");
    for ( int i = 0 ; i < 3 ; i++) {
      Serial.print (i); Serial.print(": ");
      Serial.print (rotation_byte[i]); Serial.print("\t");
    }
    Serial.print("LDR: "); Serial.println(lightVal);
    AddFile();
    previousTime = currentTime;
    Serial.println(" ");
  }
}

void AddFile() {
  String SDdata;
  SDdata += epoch; SDdata += ",";
  for ( int i = 0 ; i < 3 ; i++) {
    SDdata += TOF_byte[i]; SDdata += ",";
  }
  SDdata += acceleration[0]; SDdata += ",";
  for ( int i = 0 ; i < 3 ; i++) {
    SDdata += rotation_byte[i]; SDdata += ",";
  }
  SDdata += lightVal; SDdata += "\r\n";
  char buff[SDdata.length()];
  SDdata.toCharArray(buff, SDdata.length());
  Serial.print ( buff);
//  appendFile(SD, "/data.txt", buff);
}

void GetSensor() {
  TOF_cm[0] = (sensor1.readRangeContinuousMillimeters()) / 10;
  TOF_cm[1] = (sensor2.readRangeContinuousMillimeters()) / 10;
  TOF_cm[2] = (sensor3.readRangeContinuousMillimeters()) / 10;
  for (int i = 0 ; i < 3 ; i++) {
    if ( TOF_cm[i] > 255 ) {
      TOF_byte[i] = 0; // if value greater than 255cm
    } else {
      TOF_byte[i] = TOF_cm[i];
    }
  }
  lightVal = analogRead(LDR_PIN);
  LongByteConverter.value = lightVal;
  light_byte[0] = LongByteConverter.split[1];
  light_byte[1] = LongByteConverter.split[0];
  getSIXDOF();
}

void BLE_Notify() {
  //   notify changed value
  if (deviceConnected) {
    TOF_Characteristic->setValue(TOF_byte, 3); // 1 = 1 byte = 8 bits
    TOF_Characteristic->notify();
    ROTATION_Characteristic->setValue(rotation_byte, 3);
    ROTATION_Characteristic->notify();
    LDR_Characteristic->setValue(light_byte, 2);
    LDR_Characteristic->notify();
    ACCEL_Characteristic->setValue(acceleration , 1 );
    ACCEL_Characteristic->notify();
    delay(3); // bluetooth stack wi3ll go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
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

void getSIXDOF() {
  FreeSixIMU imu;
  float angles[3];
  imu.getYawPitchRoll(angles);
  for (int i = 0 ; i < 3 ; i++) {
    angles[i] -= IMU_cal[i];
    angles[i] += 90; //0-180 range
    if ( angles[i] > 180 ) {
      angles[i] = 180 - (angles[i] - 180 );
    } else if (angles[i] < 0) {
      angles[i] = abs (angles[i]);
    }
    rotation_byte[i] = angles[i];
  }
  short rawvalues[6];
  imu.getRawValues(rawvalues);
  acceleration[0] = (uint8_t)(sqrt(pow(rawvalues[0], 2) + pow(rawvalues[1], 2) + pow(rawvalues[2], 2)));
}

void initIMU_6DOF() {
  FreeSixIMU imu;
  imu.init();
  float angles[3];
  for (int cal_int = 0; cal_int < 500 ; cal_int ++) {//Run this code 500 times
    imu.getYawPitchRoll(angles);
    IMU_cal[0] += angles[0];
    IMU_cal[1] += angles[1];
    IMU_cal[2] += angles[2];
    delay(3);    //Delay 3us to simulate the 250Hz program loop
  }
  for (int i = 0 ; i < 3 ; i++) {
    IMU_cal[i] /= 500;
  }
  Serial.println("IMU Calibration DONE");
}
void Sensor_Init() {
  pinMode(TOF_1, OUTPUT); pinMode(TOF_2, OUTPUT); pinMode(TOF_3, OUTPUT);
  digitalWrite(TOF_1, LOW); digitalWrite(TOF_2, LOW); digitalWrite(TOF_3, LOW);
  delay(500);
  pinMode(TOF_1, INPUT); delay(150);
  sensor1.init(true); delay(100);
  sensor1.setAddress((uint8_t)23); //Set add at 0x23
  pinMode(TOF_2, INPUT); delay(150);
  sensor2.init(true); delay(100);
  sensor2.setAddress((uint8_t)24); //Set add at 0x24
  pinMode(TOF_3, INPUT); delay(150);
  sensor3.init(true); delay(100);
  sensor3.setAddress((uint8_t)25); //Set add at 0x25
  sensor1.setTimeout(500); sensor2.setTimeout(500); sensor3.setTimeout(500);
  sensor1.startContinuous(33); sensor2.startContinuous(33); sensor3.startContinuous(33);
  initIMU_6DOF();
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
  BLEService *TIMEService = pServer->createService(TIME_SERVICE_UUID);
  BLEService *DATAService = pServer->createService(DATA_SERVICE_UUID);

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

  //Create a acceleration characteristic
  ACCEL_Characteristic = MovementService->createCharacteristic(
                           ACCEL_UUID,
                           BLECharacteristic::PROPERTY_READ   |
                           BLECharacteristic::PROPERTY_WRITE  |
                           BLECharacteristic::PROPERTY_NOTIFY |
                           BLECharacteristic::PROPERTY_INDICATE
                         );
  ACCEL_Characteristic->addDescriptor(new BLE2902());

  // Create a LDR Characteristic
  LDR_Characteristic = TOFService->createCharacteristic(
                         LDR_UUID,
                         BLECharacteristic::PROPERTY_READ   |
                         BLECharacteristic::PROPERTY_WRITE  |
                         BLECharacteristic::PROPERTY_NOTIFY |
                         BLECharacteristic::PROPERTY_INDICATE
                       );
  LDR_Characteristic->addDescriptor(new BLE2902());

  // Create a BLE TOF_1 Characteristic
  TIME_Characteristic = TIMEService->createCharacteristic(
                          TIME_UUID,
                          BLECharacteristic::PROPERTY_WRITE
                        );
  TIME_Characteristic->setCallbacks(new TimeCallbacks());
  TIME_Characteristic->addDescriptor(new BLE2902());

  // Create a DATA CALL BACK
  DATA_CALL_Characteristic = DATAService->createCharacteristic(
                               DATA_CALL_UUID,
                               BLECharacteristic::PROPERTY_WRITE
                             );
  DATA_CALL_Characteristic->setCallbacks(new DataCallbacks());
  DATA_CALL_Characteristic->addDescriptor(new BLE2902());

  // Create a DATA SEND
  DATA_SEND_Characteristic = TOFService->createCharacteristic(
                               LDR_UUID,
                               BLECharacteristic::PROPERTY_READ   |
                               BLECharacteristic::PROPERTY_WRITE  |
                               BLECharacteristic::PROPERTY_NOTIFY |
                               BLECharacteristic::PROPERTY_INDICATE
                             );
  DATA_SEND_Characteristic->addDescriptor(new BLE2902());

  // Start the service
  TOFService->start();
  MovementService->start();
  TIMEService->start();
  DATAService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(TOF_SERVICE_UUID);
  pAdvertising->addServiceUUID(MOVEMENT_SERVICE_UUID);
  pAdvertising->addServiceUUID(TIME_SERVICE_UUID);
  pAdvertising->addServiceUUID(DATA_SERVICE_UUID);

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
    writeFile(SD, "/data.txt", "Epoch, TOF_1, TOF_2, TOF_3, Accel, Yaw, Pitch, Roll, LDR \r\n");
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
  } Serial.print("Read from file: ");
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
