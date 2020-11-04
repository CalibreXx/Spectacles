#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>
#include <RangeSensor.h>
#include <ComponentObject.h>
#include "SparkFun_VL53L1X.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <SparkFunLSM9DS1.h>
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include <SparkFun_RV1805.h>

// SERVICE and CHARACTERISTICS UUID for BLE
#define SENSOR_SERVICE_UUID        "efbf52a5-d22b-4808-bccd-b45c5b1d1928"
#define SENSOR_UUID        "3018bff0-ca31-430b-a6ef-dc5fefd7ee17"

#define CLOCK_SERVICE_UUID "57675859-a6f4-4445-9492-051aa8514552"
#define DATE_UUID "10ccece5-e44b-4502-8b69-09646d4072e1"

#define DATA_SERVICE_UUID "b8ec9f13-81e2-489f-b736-f4e440c86e03"
#define DATA_CALL_UUID "5022e570-0f19-4357-848a-fc74234b1348" // Write to this uuid to req for data xfer
#define DATA_SEND_UUID "38ca7184-8eeb-481f-9197-2c106f076031"

//XSHUT OF TOF
#define TOF_1 25 //0x23 LEFT
#define TOF_2 26 //0x24 CENTRE
#define TOF_3 27 //0x25 RIGHT
#define AL_ADDR 0x48

//TOF Private Variables
SFEVL53L1X sensor1; SFEVL53L1X sensor2; SFEVL53L1X sensor3;

byte TOF_byte[3] = {0, 0, 0}; // 1 byte each sensor limited to 255cm range

// LDR
SparkFun_Ambient_Light light(AL_ADDR);
uint16_t lightVal;
byte light_byte[2] = { 0 , 0 }; // 2 bytes range from 0 to 65,535

//ICM20948 Private Variables
float IMU_cal[2] = {0, 0};
byte rotation_byte[3] = {0, 0}; // Yaw Pitch Roll
byte accel_byte[3] = {0, 0, 0};
LSM9DS1 imu;
float previousTime_9DOF;
float currentTime_9DOF;

//BLE Private Variables
BLEServer* pServer = NULL;
BLECharacteristic* Sensor_Characteristic = NULL;
BLECharacteristic* Date_Characteristic = NULL;
BLECharacteristic* DATA_CALL_Characteristic = NULL;
BLECharacteristic* DATA_SEND_Characteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;

bool TimeUpdate = false ; //false = nth
bool SDsend = false;
short newTime[6];

//RTC
unsigned long epoch;
RV1805 rtc;
String receivedTime;

const uint16_t loopInterval = 1500;// 15s
unsigned long previousTime = 0 ;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.print(F("BLE C"));
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      SDsend = false;
      Serial.print(F("BLE DC"));
    }
};

class Date_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *Date_Characteristic) {
      std::string value = Date_Characteristic->getValue();
      String sentence;
      uint8_t count;
      if (value.length() > 0 ) {
        for ( uint8_t i = 1 ; i < value.length() ; i++) {
          sentence += value[i];
        }
      }
      receivedTime = sentence;
      TimeUpdate = true;
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
  uint8_t accelx_SD;
  uint8_t accely_SD;
  uint8_t accelz_SD;
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
  }
}

void loop()
{
  if (SDsend == true) {
    Serial.println(millis());
    Serial.println(F("Send SD"));
    readFile(SD, "/datalog.dat");
    SDsend = false;
    Serial.println(millis());
  }
  else if (TimeUpdate == true) {
    char buff[receivedTime.length()];
    receivedTime.toCharArray(buff, receivedTime.length());
    char *strings[7];
    char *ptr = NULL;
    byte index = 0;
    ptr = strtok(buff, ":");  // takes a list of delimiters
    while (ptr != NULL)
    {
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, ":;");  // takes a list of delimiters
    }
    for (uint8_t i = 0 ; i < index ; i++) {
      newTime[i] = atoi(strings[i]); //dd mm yy hh mm ss
    }

    rtc.setTime(0, newTime[5], newTime[4] , newTime[3], newTime[0], newTime[1], newTime[2], 1);//hund, sec, minute, hour, date, month, year, day
    Serial.println(F("TIME Up"));
    rtc.updateTime();
    epoch = rtc.getEpoch();
    epoch += 3600 * 4;
    TimeUpdate = false;
  }
  else if (millis() - previousTime >= loopInterval) {
    previousTime = millis();
    GetSensor();
    rtc.updateTime();
    epoch = rtc.getEpoch();
    epoch += 3600 * 4;
        printSensor();
    BLE_Notify();
    //    AddFile(SD , "/datalog.dat");
    //    AddFile_Txt();
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

  for ( uint8_t i = 0 ; i < 1 ; i++) {
    Serial.print(" ");
    Serial.print(rotation_byte[i]);
  }
  Serial.println("");
  Serial.print("Acceleration:      ");
  Serial.print (accel_byte[0]);
  Serial.print ("   ");
  Serial.print (accel_byte[1]);
  Serial.print ("   ");
  Serial.print (accel_byte[2]);
  Serial.print ("   ");
  Serial.print("Epoch:    ");
  Serial.println(epoch);
}

void GetSensor() {
  struct splitLong LongByteConverter;
  unsigned int TOF_cm[3] = {0, 0, 0};
  short rotation_sum[2] = {0, 0}; // Yaw Pitch Roll
  //  float acceleration_sum =  0;
  //  unsigned long previous_sampleTime = 0;
  light.powerOn();

  //  for ( uint8_t j = 0 ; j < 4 ; j++) { //take reading 4 times at a fixed interval
  //    while ( millis() - previous_sampleTime < loopInterval) {
  //      // DO NTH
  //    }
  //  previous_sampleTime = millis();
  sensor1.startRanging(); sensor2.startRanging(); sensor3.startRanging();
  TOF_cm[0] += (sensor1.getDistance()) / 10;       // TOF
  TOF_cm[1] += (sensor2.getDistance()) / 10;
  TOF_cm[2] += (sensor3.getDistance()) / 10;
  sensor1.clearInterrupt(); sensor2.clearInterrupt(); sensor3.clearInterrupt();
  sensor1.stopRanging(); sensor2.stopRanging(); sensor3.stopRanging();

  imu.readGyro();
  imu.readAccel();

  float gyroX, gyroY, gyroZ;

  float accelXYZ[3];//-20 to 20
  accelXYZ[0] = imu.calcAccel(imu.ax) * 10 ; //g's
  accelXYZ[1]  = imu.calcAccel(imu.ay) * 10 ;
  accelXYZ[2] = imu.calcAccel(imu.az) * 10 ;
  gyroX = imu.calcGyro( imu.gx);// degrees per second
  gyroY = imu.calcGyro( imu.gy) ;
  gyroZ = imu.calcGyro( imu.gz) ;

  //Calculating Yaw Pitch
  float accelRoll = atan2 (accelXYZ[1], accelXYZ[2]) * 180 / PI;
  float accelPitch = atan2 ( -accelXYZ[0], sqrt ( accelXYZ[1] * accelXYZ[1] + accelXYZ[2]  * accelXYZ[2] )) * 180 / PI; // if change here rmb to change calibration

  previousTime_9DOF = currentTime_9DOF;
  currentTime_9DOF = millis();
  float elapsedTime = (currentTime_9DOF - previousTime_9DOF) / 1000;
  float gyroRoll = gyroX * elapsedTime;
  float gyroPitch = gyroY * elapsedTime;

  rotation_sum[0] = (0.96 * accelRoll + 0.04 * gyroRoll) - IMU_cal[0];
  rotation_sum[1] = -((0.96 * accelPitch + 0.04 * gyroPitch ) - IMU_cal[1]);
  //    acceleration_sum += sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));

  for (uint8_t i = 0 ; i < 3 ; i ++) {
    accel_byte[i] = accelXYZ[i] + 30;
  }
  for (uint8_t i = 0 ; i < 2 ; i++) {
    if (rotation_sum[i] + 90 > 180) {
      rotation_byte[i] = 90;
    }  else {
      rotation_byte[i] = rotation_sum[i] + 90;
    }
  }

  for (uint8_t i = 0 ; i < 3 ; i++) {
    if ( TOF_cm[i] > 254 ) {
      TOF_cm[i] = 255;
      TOF_byte[i] = 255; // if value greater than 255cm
    } else {
      TOF_byte[i] = TOF_cm[i];
    }
  }

  long luxVal = light.readLight();
  lightVal = luxVal;
  LongByteConverter.value = luxVal;
  light_byte[0] = LongByteConverter.split[1];
  light_byte[1] = LongByteConverter.split[0];


  light.shutDown();
}

void Sensor_Init() {
  pinMode(TOF_1, OUTPUT); pinMode(TOF_2, OUTPUT); pinMode(TOF_3, OUTPUT);
  digitalWrite(TOF_1, LOW); digitalWrite(TOF_2, LOW); digitalWrite(TOF_3, LOW);
  delay(100);
  pinMode(TOF_1, INPUT); delay(150);
  sensor1.init(); delay(100);
  sensor1.setI2CAddress((uint8_t)23); //Set add at 0x23

  pinMode(TOF_2, INPUT); delay(150);
  sensor2.init(); delay(100);
  sensor2.setI2CAddress((uint8_t)24); //Set add at 0x24

  pinMode(TOF_3, INPUT); delay(150);
  sensor3.init(); delay(100);
  sensor3.setI2CAddress((uint8_t)29); //Set add at 0x25
  sensor1.setTimingBudgetInMs(50); sensor2.setTimingBudgetInMs(50); sensor3.setTimingBudgetInMs(50);
  //timing budget = ranging period
  sensor1.setIntermeasurementPeriod(300); sensor2.setIntermeasurementPeriod(100); sensor3.setIntermeasurementPeriod(100);
  // intermeasurementPeriod - timing budget = time not doing ranging
  initLight();
  initIMU_6DOF();
}

void initLight() {
  if (!light.begin()) {
    Serial.println("Could not communicate with the light sensor!");
  }
  light.setGain(0.125); // Possible values: .125, .25, 1, 2
  light.setIntegTime(100);
  light.setPowSavMode(1);
  // This will power down the sensor and the sensor will draw 0.5 micro-amps of power while shutdown.
  // light.shutDown();
  // light.powerOn();
  delay(250);
}
void initIMU_6DOF() {
  imu.settings.gyro.enabled = true;
  imu.settings.gyro.scale = 245; //set gyro range to +/-2000dps
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.lowPowerEnable = false;
  imu.settings.gyro.sampleRate = 4; //59.5HZ
  imu.settings.gyro.HPFEnable = true;
  imu.settings.gyro.HPFCutoff = 1;
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952


  imu.settings.accel.enabled = true;
  imu.settings.accel.enableX = true;
  imu.settings.accel.enableY = true;
  imu.settings.accel.enableZ = true;
  imu.settings.accel.scale = 16; //set accel scle +/-8g accel scale can be 2, 4, 8, or 16
  imu.settings.accel.sampleRate = 4; // Set accel to 10Hz.
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.bandwidth = 0;
  imu.settings.accel.highResEnable = false;
  imu.settings.accel.highResBandwidth = 0;

  imu.settings.mag.enabled = false;
  imu.settings.mag.operatingMode = 2; //0 = continuous, 1 = single, 2 = power down
  imu.settings.temp.enabled = false;
  if (imu.begin() == false) {
    Serial.println(F("9DOF failed"));
  }

  for (int cal_int = 0; cal_int < 250 ; cal_int ++) {//Run this code 250 times to calibrate gyroscope
    imu.readGyro();
    imu.readAccel();

    float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
    accelX = imu.calcAccel(imu.ax) ;
    accelY = imu.calcAccel(imu.ay) ;
    accelZ = imu.calcAccel(imu.az) ;
    gyroX = imu.calcGyro( imu.gx) ;
    gyroY = imu.calcGyro( imu.gy) ;
    gyroZ = imu.calcGyro( imu.gz) ;

    float accelRoll = atan2 (accelY, accelZ) * 180 / PI;
    float accelPitch = atan2 ( -accelX, sqrt ( accelY * accelY + accelZ * accelZ)) * 180 / PI;

    previousTime_9DOF = currentTime_9DOF;
    currentTime_9DOF = millis();
    float elapsedTime = (currentTime_9DOF - previousTime_9DOF) / 1000;
    float gyroRoll = gyroX * elapsedTime;
    float gyroPitch = gyroY * elapsedTime;

    IMU_cal[0] += 0.96 * accelRoll + 0.04 * gyroRoll;
    IMU_cal[1] += 0.96 * accelPitch + 0.04 * gyroPitch;
    delay(3);    //Delay 3us to simulate the 250Hz program loop
  }
  for (uint8_t i = 0 ; i < 2 ; i++) {
    IMU_cal[i] /= 250;
  }
  Serial.println(F("IMU Calibration DONE"));
}

/* BLE FUNCTIONS */
void BLE_Notify() {
  //   notify changed value
  if (deviceConnected) {
    byte Sensor_Byte[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    Sensor_Byte[0] = TOF_byte[0];
    Sensor_Byte[1] = TOF_byte[1];
    Sensor_Byte[2] = TOF_byte[2];
    Sensor_Byte[3] = accel_byte[0];
    Sensor_Byte[4] = accel_byte[1];
    Sensor_Byte[5] = accel_byte[2];
    Sensor_Byte[6] = rotation_byte[0]; //roll
    Sensor_Byte[7] = rotation_byte[1]; //pitch
    Sensor_Byte[8] = light_byte[1];
    Sensor_Byte[9] = light_byte[0];
    Sensor_Characteristic-> setValue ( Sensor_Byte, 10);
    Sensor_Characteristic->notify();

    delay(2); // bluetooth stack wi3ll go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(250); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println(F("start advertising"));
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
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *SensorService = pServer->createService(SENSOR_SERVICE_UUID);
  BLEService *ClockService = pServer->createService(CLOCK_SERVICE_UUID);
  BLEService *DATAService = pServer->createService(DATA_SERVICE_UUID);

  Sensor_Characteristic = SensorService->createCharacteristic(
                            SENSOR_UUID,
                            BLECharacteristic::PROPERTY_READ   |
                            BLECharacteristic::PROPERTY_NOTIFY
                          );
  Sensor_Characteristic->addDescriptor(new BLE2902());

  // Create a BLE Time Characteristic
  Date_Characteristic = ClockService->createCharacteristic(
                          DATE_UUID,
                          BLECharacteristic::PROPERTY_WRITE
                        );
  Date_Characteristic->setCallbacks(new Date_Callbacks());
  Date_Characteristic->addDescriptor(new BLE2902());

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
  SensorService->start();
  ClockService->start();
  DATAService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(SENSOR_SERVICE_UUID);
  pAdvertising->addServiceUUID(CLOCK_SERVICE_UUID);
  pAdvertising->addServiceUUID(DATA_SERVICE_UUID);

  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter\

  BLEDevice::setMTU(150);
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
    writeFile(SD, "/dataHeaders.txt", "Epoch, TOF_1, TOF_2, TOF_3, Accel, Yaw, Pitch, LDR \r\n");
  }
  file.close();
}
//
//void AddFile_Txt() {
//  String dataMessage = String(epoch) + "," + String(TOF_byte[0]) + "," + String(TOF_byte[1]) + "," + String(TOF_byte[2]) + "," +
//                       String(acceleration[0]) + "," + String(rotation_byte[0]) + "," + String(rotation_byte[1]) + ","   + String(lightVal) + "\r\n";
//  Serial.print(F("Save data: "));
//  Serial.println(dataMessage);
//  appendFile(SD, "/dataHeaders.txt", dataMessage.c_str());
//  delay(200);
//}

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

void AddFile(fs::FS & fs, const char * path) {
  struct dataStore myData;
  File file = fs.open(path, FILE_APPEND);

  myData.epochTime_SD = epoch;
  myData.tof1_SD = TOF_byte[0];
  myData.tof2_SD = TOF_byte[1];
  myData.tof3_SD = TOF_byte[2];
  myData.accelx_SD = accel_byte[0];
  myData.accely_SD = accel_byte[1];
  myData.accelz_SD = accel_byte[2];
  myData.pitch_SD = rotation_byte[1];
  myData.roll_SD = rotation_byte[0];
  myData.ldr_SD = lightVal;

  Serial.println(F("Save data: "));
  file.write((const uint8_t *)&myData, sizeof(myData));
  delay(100);
  file.close();
}
void readFile(fs::FS &fs, const char * path) {
  File file = fs.open(path, FILE_READ);
  struct splitFiveLong FiveByteConverter;
  struct dataStore myData;
  struct splitLong LongByteConverter;
  int counter = 0;
  bool sendNow = false;
  byte SDData_Byte[10];

  while ( file.available()) {
    counter += 1;
    file.read((uint8_t *)&myData, sizeof(myData));
    FiveByteConverter.value = myData.epochTime_SD;
    SDData_Byte[0] = FiveByteConverter.split[4]; //EPOCH
    SDData_Byte[1] = FiveByteConverter.split[3];
    SDData_Byte[2] = FiveByteConverter.split[2];
    SDData_Byte[3] = FiveByteConverter.split[1];
    SDData_Byte[4] = FiveByteConverter.split[0];
    SDData_Byte[5] = myData.tof1_SD; //TOF
    SDData_Byte[6] = myData.tof2_SD;
    SDData_Byte[7] = myData.tof3_SD;
    SDData_Byte[8] = myData.accelx_SD; // ACCEL
    SDData_Byte[9] = myData.accely_SD; // ACCEL
    SDData_Byte[10] = myData.accelz_SD; // ACCEL
    SDData_Byte[11] = myData.pitch_SD;
    SDData_Byte[12] = myData.roll_SD;
    LongByteConverter.value =  myData.ldr_SD;
    SDData_Byte[13] = LongByteConverter.split[1]; //LDR
    SDData_Byte[14] = LongByteConverter.split[0];

    if (deviceConnected && sendNow == true ) {
      delay(3);
      DATA_SEND_Characteristic->setValue(SDData_Byte, 15); // 1 = 1 byte = 8 bits
      DATA_SEND_Characteristic->notify();
    }
    if (!deviceConnected) {
      Serial.println(F("Device disconnected"));
      delay(100); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println(F("start advertising"));
      oldDeviceConnected = deviceConnected;
      SDsend = false;
      return;
    }
    sendNow = !sendNow;
  }

  Serial.println(counter);
  Serial.println(F("Done Sending"));
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
