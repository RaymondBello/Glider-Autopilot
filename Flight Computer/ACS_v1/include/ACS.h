#pragma once

#include <Arduino.h>
#include "config.h"
#include "tools/std.h"
#include "state.h"

/**
 * @brief Mode Enum for Various Attitude Control System States
 * 
 */
enum Mode
{
  Uninitialized   = 1 << 0,
  Initialization  = 1 << 1,
  Active          = 1 << 2,
  Idle            = 1 << 3
};

/**
 * @brief Attitude Control System Class
 * 
 */
class ACS
{
private:
  /* data */
  uint8_t arming_status;    ///< arming status
  bool motors_on;           ///< motor status
  bool kill_throttle;       ///< allow autopilot to use throttle
  bool in_flight;           ///< in flight status
  bool launch;              ///< request launch
  bool use_rc;              ///< enable/disable RC input
  bool power_switch;        ///< enable/disable power from power switch (if any)
  bool ground_detected;     ///< automatic detection of landing
  bool detect_ground_once;  ///< enable automatic detection of ground (one shot)

public:
  Mode mode;             ///< current autopilot mode
  State state;
  Radio receiver;
  uint16_t flight_time;     ///< flight time in seconds
  uint16_t throttle;          ///< throttle level as will be displayed in GCS

  ACS();
  ~ACS();
  void setupMCU();
  void setupReceiver() { receiver.radioSetup(); };
  void loop();
};

/**
 * @brief Constructor
 * 
 */
ACS::ACS(/* args */)
{
  this->mode = Uninitialized;
}


void ACS::setupMCU() {

  this->state.stateInit();

  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //Set Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  
}


void ACS::loop() {

  switch(this->mode)
    {
      case Uninitialized:
        delay(1000);
        Serial.println("Uninitialized");

        this->setupMCU();
        this->setupReceiver();
        
        this->mode = Initialization;
        break;

      case Initialization:
        Serial.println("Initialization");
        break;

      case Active:
        Serial.println("Active");
        break;

      case Idle:
        Serial.println("Idle");
        break;

      default:
        Serial.println("Error");
        break;
    }
}

/**
 * @brief Destroy the ACS::ACS object
 * 
 */
ACS::~ACS()
{
}