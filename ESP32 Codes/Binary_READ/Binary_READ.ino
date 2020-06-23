#include "FS.h"
#include "SD.h"
#include "SPI.h"

File file;

struct datastore {
  uint16_t adc1;
  uint16_t adc2;
  float voltage;
  float current;
};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println(F("No SD card attached"));
    return;
  }
  // If the data.txt file doesn't exist, Create a file on the SD card and write the data labels
  file = SD.open("/datalog.dat", FILE_READ);
  //  readFile(SD, "/test1.txt");
}


void loop() {
  if (file.available()) {
    struct datastore myData;
    file.read((uint8_t *)&myData, sizeof(myData));
    Serial.print(myData.adc1, 2);
    Serial.print(" ");
    Serial.print(myData.adc2, 2);
    Serial.print(" ");
    Serial.print(myData.voltage, 4);
    Serial.print(" ");
    Serial.println(myData.current, 4);
    delay(50);
  }
}
