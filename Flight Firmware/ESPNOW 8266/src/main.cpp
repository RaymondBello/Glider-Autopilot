#include <Arduino.h>
#include <SoftwareSerial.h>

#define BAUD_RATE 57600

SoftwareSerial swSer;
int i = 0;
char payload[100];

void setup()
{
  Serial.begin(115200);
  swSer.begin(BAUD_RATE, SWSERIAL_8N1, D1, D2, false, 95, 11);
}

void loop()
{
  swSer.println("1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,\n");

  i++;
  delay(5);
}