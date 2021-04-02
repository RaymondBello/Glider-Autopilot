#include <Arduino.h>
#include <config.h>
#include <TinyGPS++.h>
#include <PWMServo.h>
#include <I2Cdev.h>
#include <Wire.h>

#include "Config.h"
#include "State.h"
#include "Radio.h"
#include "ACS.h"
#include "Cli.h"
#include "tools/std.h"
// #include "tools/radio_comms.h"
#include "tools/kalman_filter.h"


ACS AttitudeControlSystem;


int main() {

  Serial.begin(SERIAL_BAUD);

  while (1) {
    
    AttitudeControlSystem.loop();
    delay(1000);

  }
  return 0;
}

