#include <Arduino.h>
#include "Config.h"
#include "ACS.h"


ACS AttitudeControlSystem;


int main() {

  Serial.begin(SERIAL_BAUD);

  while (1) {
    AttitudeControlSystem.loop();
  }
  return 0;
}

