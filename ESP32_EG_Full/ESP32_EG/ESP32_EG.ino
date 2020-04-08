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
unsigned short lightVal;
byte light_byte[2] = { 0 , 0 }; // 2 bytes range from 0 to 65,535

//ICM20948 Private Variables
float IMU_cal[3] = {0, 0, 0};
byte rotation_byte[3] = {0, 0, 0}; // Yaw Pitch Roll
byte acceleration[1] = {0};
FreeSixIMU imu;

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

bool TimeUpdate = false;
bool SDsend = false;
int newTime[6]; //DD MM HH MM SS YYYY
uint32_t value = 0;

//RTC
unsigned long epoch;
RTC_DS3231 rtc;

const unsigned short loopInterval = 1000;
unsigned long previousTime = 0 ;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.print(F("Device Connected:"));
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      SDsend = false;
      Serial.print(F("Device Disconnected"));
    }
};
class TimeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *TIME_Characteristic) {
      std::string value = TIME_Characteristic->getValue();
      int BLETime[15]; //day month hour, min, sec , yyyy dayof the week
      uint8_t j = 0;
      if (value.length() == 14 ) {
        for ( uint8_t i = 0 ; i < value.length() ; i++) {
          if ( value[i] >= '0' && value[i] <= '9') {
            BLETime[j] = value[i] - '0';
            j++;
          }
        }
        newTime[0] = BLETime[0] * 10 + BLETime[1]; //dd
        newTime[1] = BLETime[2] * 10 + BLETime[3]; //mm
        newTime[5] = BLETime[4] * 1000 + BLETime[5] * 100 + BLETime[6] * 10 + BLETime[7]; //year
        newTime[2] = BLETime[8] * 10 + BLETime[9]; //hh
        newTime[3] = BLETime[10] * 10 + BLETime[11]; //mm
        newTime[4] = BLETime[12] * 10 + BLETime[13]; //ss
        for ( uint8_t i = 0 ; i < 6 ; i++) {
          Serial.print(newTime[i]);
        }
        TimeUpdate = true;
      }
    }
};
class DataCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *DATA_CALL_Characteristic) {
      std::string value = TIME_Characteristic->getValue();
      SDsend = true;
    }
};

struct splitLong { //split long into 2 byte sized packets
  union {
    long value;
    char split[2];
  } __attribute__((packed));
};
struct splitLong LongByteConverter;

struct splitFiveLong { //split long into 5 byte sized packets for epoch time
  union {
    long value;
    char split[5];
  } __attribute__((packed));
};
struct splitFiveLong FiveByteConverter;

void setup()
{
  Wire.begin();
  Serial.begin (115200);
  Sensor_Init();
  BLE_Init();
  SD_Init();
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1);
  }
}

void loop()
{
  if (SDsend == true) {
    Serial.println(millis());
    Serial.println("HERE:");
    Serial.println(F("Reading file"));
    readFile(SD, "/data.txt");
    SDsend = false;
    Serial.println(millis());
  }
  else if (TimeUpdate == true) {
    rtc.adjust(DateTime(newTime[5], newTime[1], newTime[0], //yy month dd
                        newTime[2], newTime[3], newTime[4])); //hh mm ss
    Serial.println(F("Adjust Time completed"));
    TimeUpdate = false;
  }
  else if (millis() - previousTime >= loopInterval) {
    previousTime = millis();
    DateTime now = rtc.now();
    epoch = now.unixtime();
    Serial.println(epoch);
    GetSensor();
    BLE_Notify();
    AddFile();
    //    Serial.print("Epoch: "); Serial.println(epoch);
    //    Serial.print("TOF ");
    //    for ( uint8_t i = 0 ; i < 3 ; i++) {
    //      Serial.print (i); Serial.print(": ");
    //      Serial.print(TOF_byte[i]); Serial.print("\t");
    //    }
    //    Serial.print ( "Acceleration: "); Serial.println(acceleration[0]);
    //    Serial.print("Rotation ");
    //    for ( uint8_t i = 0 ; i < 3 ; i++) {
    //      Serial.print (i); Serial.print(": ");
    //      Serial.print (rotation_byte[i]); Serial.print("\t");
    //    }
    //    Serial.print("LDR: "); Serial.println(lightVal);
    //    Serial.println(" ");
  }
}

/* Sensor FUNCTIONS */
void GetSensor() {
  TOF_cm[0] = (sensor1.readRangeContinuousMillimeters()) / 10;
  TOF_cm[1] = (sensor2.readRangeContinuousMillimeters()) / 10;
  TOF_cm[2] = (sensor3.readRangeContinuousMillimeters()) / 10;
  for (uint8_t i = 0 ; i < 3 ; i++) {
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
void getSIXDOF() {
  float angles[3];
  imu.getYawPitchRoll(angles);
  for (uint8_t i = 0 ; i < 3 ; i++) {
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
void initIMU_6DOF() {
  imu.init(true);
  float angles[3];
  for (int cal_int = 0; cal_int < 500 ; cal_int ++) {//Run this code 500 times
    imu.getYawPitchRoll(angles);
    IMU_cal[0] += angles[0];
    IMU_cal[1] += angles[1];
    IMU_cal[2] += angles[2];
    delay(3);    //Delay 3us to simulate the 250Hz program loop
  }
  for (uint8_t i = 0 ; i < 3 ; i++) {
    IMU_cal[i] /= 500;
  }
  Serial.println(F("IMU Calibration DONE"));
}

/* BLE FUNCTIONS */
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
void BLE_Init() { //Server --> Service --> Characteristics <-- sensor data input
  // Create the BLE Device
  BLEDevice::init("ESP32-BRYAN");
  BLEDevice::setMTU(23);
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
  DATA_SEND_Characteristic = DATAService->createCharacteristic(
                               DATA_SEND_UUID,
                               BLECharacteristic::PROPERTY_READ   |
                               BLECharacteristic::PROPERTY_NOTIFY
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
  Serial.println(F("Waiting a client connection to notify..."));
}

/* SD Card functions */
void SD_Init() {
  if (!SD.begin()) {
    Serial.println(F("Card Mount Failed"));
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println(F("No SD card attached"));
    return;
  }
  // If the data.txt file doesn't exist, Create a file on the SD card and write the data labels
  File file = SD.open("/data.txt");
  if (!file) {
    Serial.println(F("Creating file..."));
    writeFile(SD, "/data.txt", "Epoch, TOF_1, TOF_2, TOF_3, Accel, Yaw, Pitch, Roll, LDR \r\n");
  }
  file.close();
}
void AddFile() {
  String dataMessage = String(epoch) + "," + String(TOF_byte[0]) + "," + String(TOF_byte[1]) + "," + String(TOF_byte[2]) + "," +
                       String(acceleration[0]) + "," + String(rotation_byte[0]) + "," + String(rotation_byte[1]) + "," + String(rotation_byte[2]) + ","
                       + String(lightVal) + "\r\n";
  Serial.print(F("Save data: "));
  Serial.println(dataMessage);
  appendFile(SD, "/data.txt", dataMessage.c_str());
  delay(200);
}
void readFile(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  int i = 0;
  String sentence;
  SDsend = true;
  while (file.available() && SDsend == true && deviceConnected) {
    char currChar = file.read();
    if ( i > 50 ) { // remove header
      if (currChar == '\r') {
      }
      if (currChar == '\n') {
        int noCommas[8];
        uint8_t k = 0;
        for ( int j = 0 ; j < sentence.length() ; j++) {
          if (sentence[j] == ',') {
            noCommas[k] = j;
            k++;
          }
        }// Search for commas
        unsigned int SensorSDData[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0}; //epoch, tof1,2,3 accel, yaw pitch roll, ldr
        for ( uint8_t j = 0 ; j < noCommas[0] ; j++) { // first commma
          SensorSDData[0] += (sentence[j] - '0') * pow(10, noCommas[0] - (j + 1)) ;
        }
        for ( uint8_t p = 1 ; p < 8 ; p++) {
          for ( uint8_t j = noCommas[p - 1] + 1 ; j < noCommas [p] ; j++) {
            SensorSDData [p] += (sentence[j] - '0' ) * pow (10, (noCommas[p]  - j  - 1));
          }
        }
        for ( uint8_t j = noCommas[7] + 1 ; j < sentence.length() - 1 ; j++) { //TOF3
          SensorSDData[8] += (sentence[j] - '0') * pow (10, ( sentence.length() - 1  - j   - 1 ) );
        }
        byte SDData_Byte[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0};
        FiveByteConverter.value = SensorSDData[0];
        SDData_Byte[0] = FiveByteConverter.split[4];
        SDData_Byte[1] = FiveByteConverter.split[3];
        SDData_Byte[2] = FiveByteConverter.split[2];
        SDData_Byte[3] = FiveByteConverter.split[1];
        SDData_Byte[4] = FiveByteConverter.split[0];
        for (uint8_t j = 1 ; j < 4 ; j++) {
          if ( SensorSDData[j] > 255 ) {
            SDData_Byte[j + 4] = 0; // if value greater than 255cm
          } else {
            SDData_Byte[j + 4] = SensorSDData[j];
          }
        }
        SDData_Byte[8] = SensorSDData[4]; // accel
        SDData_Byte[9] = SensorSDData[5];
        SDData_Byte[10] = SensorSDData[6];
        SDData_Byte[11] = SensorSDData[7];
        LongByteConverter.value = SensorSDData[8];
        SDData_Byte[12] = LongByteConverter.split[1];
        SDData_Byte[13] = LongByteConverter.split[0];
        if (deviceConnected) {
          delay(3);
          DATA_SEND_Characteristic->setValue(SDData_Byte, 14); // 1 = 1 byte = 8 bits
          DATA_SEND_Characteristic->notify();

        }
        if (!deviceConnected) {
          delay(100); // give the bluetooth stack the chance to get things ready
          pServer->startAdvertising(); // restart advertising
          Serial.println("start advertising");
          oldDeviceConnected = deviceConnected;
          SDsend = false;
          break;
        }
        currChar = file.read();
        sentence = currChar;
      }
      else {
        sentence += currChar; //compile full sentence
      }
    }
    i++;
  }
  SDsend = false;
  file.close();
  deleteFile (SD , "/data.txt");
  SD_Init();
}

void writeFile(fs::FS & fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to open file for writing"));
    return;
  }
  if (file.print(message)) {
    Serial.println(F("File written"));
  } else {
    Serial.println(F("Write failed"));
  }
  file.close();
}

void appendFile(fs::FS & fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println(F("Failed to open file for appending"));
    return;
  }
  if (file.print(message)) {
    Serial.println(F("Message appended"));
  } else {
    Serial.println(F("Append failed"));
  }
  file.close();
}

void deleteFile(fs::FS & fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println(F("File deleted"));
  } else {
    Serial.println(F("Delete failed"));
  }
}
