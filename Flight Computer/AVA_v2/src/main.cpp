/******** Libraries **********/
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <ServoInput.h>

/******* User-defined *******/
#include "kalman_filter.h"
#include "mcu_config.h"


void setup()
{
    // Setup Indicators
    pinMode(LED_BUILTIN, OUTPUT);

    // BUILTIN LED ON during Setup
    digitalWrite(LED_BUILTIN, HIGH);
}
void loop()
{
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
}