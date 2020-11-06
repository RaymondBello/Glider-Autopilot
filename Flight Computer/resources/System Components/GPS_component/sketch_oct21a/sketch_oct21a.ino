#define TX D3                            //pin number for commands from Arduino to GPS
#define RX D4                            //pin number for serial data received from GPS
#define GPSBaud 9600                     //GPS Baud rate
#define Serial_Monitor_Baud 115200       //this is baud rate used for the Arduino IDE Serial Monitor

#include <Arduino.h>
#include <SoftwareSerial.h>                     
SoftwareSerial GPSserial(RX, TX);  


void loop()                    
{
  while (GPSserial.available() > 0)
  Serial.write(GPSserial.read());
}


void setup()
{
 Serial.begin(Serial_Monitor_Baud);       //start Serial console ouput
 GPSserial.begin(GPSBaud);                //start softserial for GPS at defined baud rate
}
