#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;


// For stats that happen every 5 seconds
unsigned long last = 0UL;

void setup()
{
  Serial.begin(115200);
  Serial8.begin(GPSBaud);

}

void loop()
{
  // Dispatch incoming characters
  while (Serial8.available() > 0)
    Serial.print(char(Serial8.read()));

  
}