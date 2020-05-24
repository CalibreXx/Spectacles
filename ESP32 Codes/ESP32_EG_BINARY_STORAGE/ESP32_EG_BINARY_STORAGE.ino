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
#define TIME_CALL_UUID "7ec7bbf4-cc3b-430b-81a3-de900b242c7a"
#define DD_UUID "10ccece5-e44b-4502-8b69-09646d4072e1"
#define MonMon_UUID "839a1c7a-d528-4001-a1f0-2e1409acbe3b"
#define YY_UUID "e8e6995b-7035-44cb-9aa4-6c4e12d0d65b"
#define HH_UUID "1d584eb9-ab30-43ae-a83f-a9b62241bbd2"
#define MinMin_UUID "944cd4a5-03a8-42d2-a347-3047f2ff3a83"
#define SS_UUID "8c14ac64-1f27-4268-b108-40d138fd22d4"

#define DATA_SERVICE_UUID "b8ec9f13-81e2-489f-b736-f4e440c86e03"
#define DATA_CALL_UUID "5022e570-0f19-4357-848a-fc74234b1348" // Write to this uuid to req for data xfer
#define DATA_SEND_UUID "38ca7184-8eeb-481f-9197-2c106f076031"

//XSHUT OF TOF
#define TOF_1 25 //0x23 LEFT
#define TOF_2 26 //0x24 CENTRE
#define TOF_3 27 //0x25 RIGHT

//TOF Private Variables
VL53L1X sensor1; VL53L1X sensor2; VL53L1X sensor3;

byte TOF_byte[3] = {0, 0, 0}; // 1 byte each sensor limited to 255cm range

// LDR
const int LDR_PIN = 34; // analog pins
uint16_t lightVal;
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

BLECharacteristic* Time_Call_Characteristic = NULL;
BLECharacteristic* DD_Characteristic = NULL;
BLECharacteristic* MonMon_Characteristic = NULL;
BLECharacteristic* YY_Characteristic = NULL;
BLECharacteristic* HH_Characteristic = NULL;
BLECharacteristic* MinMin_Characteristic = NULL;
BLECharacteristic* SS_Characteristic = NULL;

BLECharacteristic* DATA_CALL_Characteristic = NULL;
BLECharacteristic* DATA_SEND_Characteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;

bool TimeUpdate = false ; //false = nth
bool SDsend = false;
int newTime[6]; //DD MM HH MM SS YYYY
uint32_t value = 0;

//RTC
unsigned long epoch;
RTC_DS3231 rtc;

const uint16_t loopInterval = 5000;
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

class Time_Call_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *Time_Call_Characteristic) {
      std::string value = Time_Call_Characteristic->getValue();
      if (value.length() > 0) {
        if ( int(value[0]) == 17 && int(value[1]) == 16){
          
        }
      }
    }
};

class DD_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *DD_Characteristic) {
      std::string value = DD_Characteristic->getValue();
      int BLETime[15];
      uint8_t j = 0;
      if (value.length() > 0 ) {
        for ( uint8_t i = 0 ; i < value.length() ; i++) {
          if ( value[i] >= '0' && value[i] <= '9') {
            BLETime[j] = value[i] - '0';
            j++;
          }
        }
        newTime[0] = BLETime[0] * 10 + BLETime[1]; //dd
      }
    }
};

class MonMon_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *MonMon_Characteristic) {
      std::string value = MonMon_Characteristic->getValue();
      int BLETime[15];
      uint8_t j = 0;
      if (value.length() > 0 ) {
        for ( uint8_t i = 0 ; i < value.length() ; i++) {
          if ( value[i] >= '0' && value[i] <= '9') {
            BLETime[j] = value[i] - '0';
            j++;
          }
        }
        newTime[1] = BLETime[0] * 10 + BLETime[1]; //dd
      }
    }
};
class YY_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *YY_Characteristic) {
      std::string value = YY_Characteristic->getValue();
      int BLETime[15];
      uint8_t j = 0;
      if (value.length() > 0 ) {
        for ( uint8_t i = 0 ; i < value.length() ; i++) {
          if ( value[i] >= '0' && value[i] <= '9') {
            BLETime[j] = value[i] - '0';
            j++;
          }
        }
        newTime[2] = BLETime[0] * 1000 + BLETime[1] * 100 + BLETime[2] * 10 +  BLETime[3]; //dd
      }
    }
};
class HH_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *HH_Characteristic) {
      std::string value = HH_Characteristic->getValue();
      int BLETime[15];
      uint8_t j = 0;
      if (value.length() > 0 ) {
        for ( uint8_t i = 0 ; i < value.length() ; i++) {
          if ( value[i] >= '0' && value[i] <= '9') {
            BLETime[j] = value[i] - '0';
            j++;
          }
        }
        newTime[3] = BLETime[0] * 10 + BLETime[1]; //dd
      }
    }
};
class MinMin_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *MinMin_Characteristic) {
      std::string value = MinMin_Characteristic->getValue();
      int BLETime[15];
      uint8_t j = 0;
      if (value.length() > 0 ) {
        for ( uint8_t i = 0 ; i < value.length() ; i++) {
          if ( value[i] >= '0' && value[i] <= '9') {
            BLETime[j] = value[i] - '0';
            j++;
          }
        }
        newTime[4] = BLETime[0] * 10 + BLETime[1]; //dd
      }
    }
};
class SS_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *SS_Characteristic) {
      std::string value = SS_Characteristic->getValue();
      int BLETime[15];
      uint8_t j = 0;
      if (value.length() > 0 ) {
        for ( uint8_t i = 0 ; i < value.length() ; i++) {
          if ( value[i] >= '0' && value[i] <= '9') {
            BLETime[j] = value[i] - '0';
            j++;
          }
        }
        newTime[5] = BLETime[0] * 10 + BLETime[1];
        TimeUpdate = true;
      }
    }
};

class DataCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *DATA_CALL_Characteristic) {
      std::string value = DATA_CALL_Characteristic->getValue();
      SDsend = true;
    }
};

struct splitLong { //split long into 2 byte sized packets
  union {
    long value;
    char split[2];
  } __attribute__((packed));
};

struct splitFiveLong { //split long into 5 byte sized packets for epoch time
  union {
    long value;
    char split[5];
  } __attribute__((packed));
};

struct dataStore {
  uint32_t epochTime_SD;
  uint8_t tof1_SD;
  uint8_t tof2_SD;
  uint8_t tof3_SD;
  uint8_t accel_SD;
  uint8_t yaw_SD;
  uint8_t pitch_SD;
  uint8_t roll_SD;
  uint16_t ldr_SD;
};

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
    Serial.println(F("Reading file"));
    readFile(SD, "/datalog.dat");
    SDsend = false;
    Serial.println(millis());
  }
  else if (TimeUpdate == true) {
    rtc.adjust(DateTime(newTime[2], newTime[1], newTime[0], //yy month dd
                        newTime[3], newTime[4], newTime[5])); //hh mm ss
    Serial.println(F("Adjust Time completed"));
    TimeUpdate = false;
  }
  else if (millis() - previousTime >= loopInterval) {
    previousTime = millis();
    DateTime now = rtc.now();
    epoch = now.unixtime();
    GetSensor();
    BLE_Notify();
    AddFile(SD , "/datalog.dat");
  }
}

/* Sensor FUNCTIONS */

void printSensor() {
  Serial.print( "TOF Sensors:        ");
  for ( uint8_t i = 0 ; i < 3 ; i++) {
    Serial.print("   ");
    Serial.print(TOF_byte[i]);
  }
  Serial.println("");
  Serial.print ( "Light Val:      ");
  Serial.println(lightVal);

  for ( uint8_t i = 0 ; i < 3 ; i++) {
    Serial.print(" ");
    Serial.print(rotation_byte[i]);
  }
  Serial.println("");
  Serial.print("Acceleration:      ");
  Serial.println(acceleration[0]);

}
void GetSensor() {

  struct splitLong LongByteConverter;
  unsigned int TOF_cm[3] = {0, 0, 0};
  int light_value_sum = 0 ;
  int rotation_sum[3] = {0, 0, 0}; // Yaw Pitch Roll
  int acceleration_sum =  0;
  unsigned long previous_sampleTime = 0;

  for ( uint8_t j = 0 ; j < 4 ; j++) { //take reading 4 times at a fixed interval
    while ( millis() - previous_sampleTime < loopInterval / 4) {
      // DO NTH
    }
    previous_sampleTime = millis();

    TOF_cm[0] += (sensor1.readRangeContinuousMillimeters()) / 10;       // TOF sensor
    TOF_cm[1] += (sensor2.readRangeContinuousMillimeters()) / 10;
    TOF_cm[2] += (sensor3.readRangeContinuousMillimeters()) / 10;
    light_value_sum += analogRead(LDR_PIN);                             //Light Sensor
    float angles[3];                                                    // IMU 6D0F Sensor
    short rawvalues[6];
    imu.getYawPitchRoll(angles);
    for (uint8_t i = 0 ; i < 3 ; i++) {
      angles[i] = angles[i] - IMU_cal[i] + 90; //0-180 range
      if ( angles[i] > 180 ) {
        angles[i] = 360 - angles[i];
      } else if (angles[i] < 0) {
        angles[i] = abs (angles[i]);
      }
      rotation_sum[i] += angles[i];
    }
    imu.getRawValues(rawvalues);
    acceleration_sum += (uint8_t)(sqrt(pow(rawvalues[0], 2) + pow(rawvalues[1], 2) + pow(rawvalues[2], 2)));
  }

  for (uint8_t i = 0 ; i < 3 ; i++) {
    rotation_byte[i] = rotation_sum[i] / 4 ;
    TOF_cm[i] = TOF_cm[i] / 4;
    if ( TOF_cm[i] > 255 ) {
      TOF_cm[i] = 255;
      TOF_byte[i] = 255; // if value greater than 255cm
    } else {
      TOF_byte[i] = TOF_cm[i];
    }
  }

  lightVal = light_value_sum / 4;
  LongByteConverter.value = lightVal;
  light_byte[0] = LongByteConverter.split[1];
  light_byte[1] = LongByteConverter.split[0];
  acceleration[0] = acceleration_sum / 4;
}

void Sensor_Init() {
  pinMode(TOF_1, OUTPUT); pinMode(TOF_2, OUTPUT); pinMode(TOF_3, OUTPUT);
  digitalWrite(TOF_1, LOW); digitalWrite(TOF_2, LOW); digitalWrite(TOF_3, LOW);
  delay(100);
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
    delay(250); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    //  }
    delay(200);
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

  Time_Call_Characteristic = TIMEService->createCharacteristic(
                               TIME_CALL_UUID,
                               BLECharacteristic::PROPERTY_WRITE
                             );
  Time_Call_Characteristic->setCallbacks(new Time_Call_Callbacks());
  Time_Call_Characteristic->addDescriptor(new BLE2902());

  // Create a BLE Time Characteristic
  DD_Characteristic = TIMEService->createCharacteristic(
                        DD_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  DD_Characteristic->setCallbacks(new DD_Callbacks());
  DD_Characteristic->addDescriptor(new BLE2902());

  // Create a BLE Time Characteristic****************** TIME ****************************
  MonMon_Characteristic = TIMEService->createCharacteristic(
                            MonMon_UUID,
                            BLECharacteristic::PROPERTY_WRITE
                          );
  MonMon_Characteristic->setCallbacks(new MonMon_Callbacks());
  MonMon_Characteristic->addDescriptor(new BLE2902());

  YY_Characteristic = TIMEService->createCharacteristic(
                        YY_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  YY_Characteristic->setCallbacks(new YY_Callbacks());
  YY_Characteristic->addDescriptor(new BLE2902());

  HH_Characteristic = TIMEService->createCharacteristic(
                        HH_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  HH_Characteristic->setCallbacks(new HH_Callbacks());
  HH_Characteristic->addDescriptor(new BLE2902());

  MinMin_Characteristic = TIMEService->createCharacteristic(
                            MinMin_UUID,
                            BLECharacteristic::PROPERTY_WRITE
                          );
  MinMin_Characteristic->setCallbacks(new MinMin_Callbacks());
  MinMin_Characteristic->addDescriptor(new BLE2902());

  SS_Characteristic = TIMEService->createCharacteristic(
                        SS_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  SS_Characteristic->setCallbacks(new SS_Callbacks());
  SS_Characteristic->addDescriptor(new BLE2902());

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
  File file = SD.open("/dataHeaders.txt");
  if (!file) {
    Serial.println(F("Creating file..."));
    writeFile(SD, "/dataHeaders.txt", "Epoch, TOF_1, TOF_2, TOF_3, Accel, Yaw, Pitch, Roll, LDR \r\n");
  }
  file.close();
}
void AddFile(fs::FS & fs, const char * path) {
  struct dataStore myData;
  File file = fs.open(path, FILE_APPEND);

  myData.epochTime_SD = epoch;
  myData.tof1_SD = TOF_byte[0];
  myData.tof2_SD = TOF_byte[1];
  myData.tof3_SD = TOF_byte[2];
  myData.accel_SD = acceleration[0];
  myData.yaw_SD = rotation_byte[0];
  myData.pitch_SD = rotation_byte[1];
  myData.roll_SD = rotation_byte[2];
  myData.ldr_SD = lightVal;

  Serial.println(F("Save data: "));
  file.write((const uint8_t *)&myData, sizeof(myData));
  delay(200);
  file.close();
}
void readFile(fs::FS &fs, const char * path) {
  File file = fs.open(path, FILE_READ);
  struct splitFiveLong FiveByteConverter;
  struct dataStore myData;
  struct splitLong LongByteConverter;
  int counter = 0;

  while ( file.available()) {
    counter += 1;
    file.read((uint8_t *)&myData, sizeof(myData));
    FiveByteConverter.value = myData.epochTime_SD;
    byte SDData_Byte[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0};
    SDData_Byte[0] = FiveByteConverter.split[4]; //EPOCH
    SDData_Byte[1] = FiveByteConverter.split[3];
    SDData_Byte[2] = FiveByteConverter.split[2];
    SDData_Byte[3] = FiveByteConverter.split[1];
    SDData_Byte[4] = FiveByteConverter.split[0];
    SDData_Byte[5] = myData.tof1_SD;  //TOF
    SDData_Byte[6] = myData.tof2_SD;
    SDData_Byte[7] = myData.tof3_SD;
    SDData_Byte[8] = myData.accel_SD; // ACCEL
    SDData_Byte[9] = myData.yaw_SD;
    SDData_Byte[10] = myData.pitch_SD;
    SDData_Byte[11] = myData.roll_SD;
    LongByteConverter.value =  myData.ldr_SD;
    SDData_Byte[12] = LongByteConverter.split[1]; //LDR
    SDData_Byte[13] = LongByteConverter.split[0];

    if (deviceConnected) {
      delay(3);
      DATA_SEND_Characteristic->setValue(SDData_Byte, 14); // 1 = 1 byte = 8 bits
      DATA_SEND_Characteristic->notify();
    }
    if (!deviceConnected) {
      Serial.println("Device disconnected");
      delay(100); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
      SDsend = false;
      return;
    }
  }
  Serial.println(counter);
  Serial.println("Done Sending");
  SDsend = false;
  file.close();
  //  deleteFile (SD , path);
  //  SD_Init();
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

void deleteFile(fs::FS & fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println(F("File deleted"));
  } else {
    Serial.println(F("Delete failed"));
  }
}
