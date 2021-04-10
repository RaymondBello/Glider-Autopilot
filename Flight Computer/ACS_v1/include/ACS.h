#pragma once


#include <Arduino.h>
#include "config.h"
#include "tools/std.h"
#include "State.h"
#include "Radio.h"
#include "FC.h"

/**
 * @brief Mode Enum for Various Attitude Control System States
 * 
 */
enum Mode
{
  Error           = 0,
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
  uint8_t arming_status;    // arming status
  bool motors_on;           // motor status
  bool kill_throttle;       // allow autopilot to use throttle
  bool in_flight;           // in flight status
  bool launch;              // request launch
  bool use_rc;              // enable/disable RC input
  bool power_switch;        // enable/disable power from power switch (if any)
  bool ground_detected;     // automatic detection of landing
  bool detect_ground_once;  // enable automatic detection of ground (one shot)

public:
  Mode mode;                // current autopilot mode
  State stateACS;
  Radio receiver;
  FC flightController;
  
  ACS();
  ~ACS();
  void setupMCU();
  void setupReceiver() { receiver.radioSetup(); };
  void setupBlink(int numBlinks, int upTime, int downTime);
  void setupBeep(int numBeeps, int upTime, int downTime);
  BoolInt setupFlightController();
  BoolInt calibrateFlightController();
  void loop();
  void updateFCtime();
  void updateFCorientation();
};

/**
 * @brief Constructor
 * 
 */
ACS::ACS(/* args */)
{
  this->mode = Uninitialized;
}

void ACS::setupMCU() 
{
  this->stateACS.stateInit();

  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //Set Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
}

BoolInt ACS::setupFlightController() 
{

  this->flightController.init();
  BoolInt passErr = this->flightController.initIMU();
  
  if (passErr.flag) {
    passErr = this->flightController.initBaro();
  }
  return passErr;
}

BoolInt ACS::calibrateFlightController() 
{
  BoolInt passErr;
  this->flightController.calculateIMUerror();
  delay(10);
  this->flightController.calibrateAttitude(0);
  delay(10);
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)
  setupBeep(2, 160, 70);

  passErr.flag = 1;
  passErr.ErrCode = 0;

  return passErr;
}

void ACS::loop() 
{

  switch(this->mode)
    {
      case Uninitialized:
      {
        delay(1000);
        Serial.println("Uninitialized");
        this->setupMCU();
        this->mode = Mode::Initialization;
        break;
      }
      case Initialization:
      {
        Serial.println("Initialization");
        this->setupReceiver();
        BoolInt boolint = this->setupFlightController();
        boolint.flag ? (0) : (this->mode = Mode::Error);
        BoolInt boolint1 = this->calibrateFlightController();
        boolint1.flag ? (this->mode = Mode::Active) : (this->mode = Mode::Error);
        Serial.println("\tINFO: Boot subroutine completed\n");
        delay(1000);
        break;
      }
      case Active:
        Serial.println("Active");
        flightController.loopBlink();
        updateFCtime();
        updateFCorientation();


        flightController.loopRate(2000);
        break;
      case Idle:
        Serial.println("Idle");
        break;
      case Error:
        Serial.println("Error");
        break;
    }
}

void ACS::setupBlink(int numBlinks, int upTime, int downTime)
{
    for (int j = 1; j <= numBlinks; j++)
    {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}

void ACS::setupBeep(int numBeeps, int upTime, int downTime)
{
    for (int j = 1; j <= numBeeps; j++)
    {
        digitalWrite(BUZZER_PIN, LOW);
        delay(downTime);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(upTime);
    }
    digitalWrite(BUZZER_PIN, LOW);
}

void ACS::updateFCtime()
{
  flightController.prev_time = flightController.current_time;
  flightController.current_time = micros();
  flightController.dt = (flightController.current_time - flightController.prev_time) / 1000000.0;
}

void ACS::updateFCorientation()
{
  flightController.getIMUdata();
  flightController.Madgwick(flightController.GyroX, -flightController.GyroY, -flightController.GyroZ, -flightController.AccX, flightController.AccY, flightController.AccZ, flightController.MagY, -flightController.MagX, flightController.MagZ, flightController.dt);
}

/**
 * @brief Destroy the ACS::ACS object
 */
ACS::~ACS()
{}