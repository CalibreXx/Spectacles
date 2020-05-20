#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor1;
VL53L1X sensor2;

void setup()
{

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(13, LOW);  
  digitalWrite(12, LOW);


  delay(500);
  Wire.begin();

  Serial.begin (115200);

  pinMode(13, INPUT);
  delay(150);
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)23);

  pinMode(12, INPUT);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)24);

  sensor1.setTimeout(500);
  sensor2.setTimeout(500);

  sensor1.startContinuous(33);
  sensor2.startContinuous(33);

}

void loop()
{
  Serial.print(sensor1.readRangeContinuousMillimeters());
  Serial.print(',');
  Serial.print(sensor2.readRangeContinuousMillimeters());
  Serial.print(',');
  Serial.println();
}
