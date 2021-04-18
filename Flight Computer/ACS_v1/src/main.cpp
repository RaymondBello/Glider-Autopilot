<<<<<<< HEAD
#include <Arduino.h>
#include "Config.h"
#include "ACS.h"

=======
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const uint32_t GPSBaud = 9600;
>>>>>>> 0163e34612ea173b4881ffe5d33db35007dc7013

ACS AttitudeControlSystem;


<<<<<<< HEAD
int main() {

  Serial.begin(SERIAL_BAUD);

  while (1) 
  {
    AttitudeControlSystem.loop();
  }
  return 0;
}
=======
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
>>>>>>> 0163e34612ea173b4881ffe5d33db35007dc7013

  
}