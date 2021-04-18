#pragma once

#include <Arduino.h>
#include <SimpleCLI.h>
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
  Error = 0,
  Uninitialized = 1 << 0,
  Initialization = 1 << 1,
  Active = 1 << 2,
  Idle = 1 << 3
};

/**
 * @brief Attitude Control System Class
 * 
 */
class ACS
{
private:
  /* data */
  uint8_t arming_status;   // arming status
  bool motors_on;          // motor status
  bool kill_throttle;      // allow autopilot to use throttle
  bool in_flight;          // in flight status
  bool launch;             // request launch
  bool use_rc;             // enable/disable RC input
  bool power_switch;       // enable/disable power from power switch (if any)
  bool ground_detected;    // automatic detection of landing
  bool detect_ground_once; // enable automatic detection of ground (one shot)

public:
  unsigned long print_counter, serial_counter;
  Mode mode; // current autopilot mode
  State stateACS;
  Radio receiver;
  FC flightController;

  // CommandLine Interface
  SimpleCLI cli;
  Command cmdGet;
  Command cmdSet;
  Command cmdExec;

  // CliCommand CommandLineInterface;

  ACS();
  ~ACS();
  void setupMCU();
  void setupReceiver() { receiver.radioSetup(); };
  void setupBlink(int numBlinks, int upTime, int downTime);
  void setupBeep(int numBeeps, int upTime, int downTime);
  BoolInt setupFlightController();
  BoolInt calibrateFlightController();
  void loop();
  void updateFlightControllerTime();
  void updateFlightControllerOrientation();
  void updateFlightControllerPIDloop();
  void printDebugMessages();
  void printRadioData();
  void printIMUdata();
  void printBMPdata();
  void printDesiredState();
  void printRollPitchYaw();
  void printPIDoutput();
  void printMotorCommands();
  void printServoCommands();
  void printPIDvalues();
  void printAircraftState();

  static void execCallback(cmd *execmd);
  void handleSerial();
  void handleReceivedMessage(char *msg);
  void processGetCommand(const char *setting);
  void processSetCommand(const char *setting, const char *value);
};

/**
 * @brief Constructor
 * 
 */
ACS::ACS(/* args */)
{
  mode = Uninitialized;
  cmdGet = cli.addCommand("get");
  cmdGet.addPositionalArgument("setting");
  // cmdGet.addPositionalArgument("value");

  cmdSet = cli.addCommand("set");
  cmdSet.addPositionalArgument("setting");
  cmdSet.addPositionalArgument("val/ue");

  cmdExec = cli.addBoundlessCommand("exec", execCallback);
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

  if (passErr.flag)
  {
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

  switch (this->mode)
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
    Serial.println("INFO, ACS-Mode: Initialization");
    this->setupReceiver();
    BoolInt boolint = this->setupFlightController();
    boolint.flag ? (0) : (this->mode = Mode::Error);
    BoolInt boolint1 = this->calibrateFlightController();
    boolint1.flag ? (this->mode = Mode::Active) : (this->mode = Mode::Error);
    Serial.println("INFO, ACS boot SR completed");
    delay(1000);
    break;
  }
  case Active:
  {
    // Serial.println("Active");
    flightController.loopBlink();
    updateFlightControllerTime();
    updateFlightControllerOrientation();
    updateFlightControllerPIDloop();
    printDebugMessages();

    /** Handle CLI message **/
    // Serial.println("Wrote to Motors");
    
    handleSerial();

    flightController.loopRate(2000);
    break;
  }
  case Idle:
  {
    Serial.println("INFO, ACS-Mode: Idle");
    break;
  }
  case Error:
  {
    Serial.println("INFO, ACS-Mode: Error");
    flightController.loopBeep();
    break;
  }
  default:
  {
    Serial.println("FATAL, FATAL ACS-Mode Error! Rebooting...");
    this->setupBeep(4, 160, 70);
    delay(50);
    SCB_AIRCR = 0x05FA0004;
    break;
  }
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

void ACS::updateFlightControllerTime()
{
  flightController.prev_time = flightController.current_time;
  flightController.current_time = micros();
  flightController.dt = (flightController.current_time - flightController.prev_time) / 1000000.0;
}

void ACS::updateFlightControllerOrientation()
{
  flightController.getIMUdata();
  flightController.Madgwick(flightController.GyroX, -flightController.GyroY, -flightController.GyroZ, -flightController.AccX, flightController.AccY, flightController.AccZ, flightController.MagY, -flightController.MagX, flightController.MagZ, flightController.dt);
  flightController.getBMPdata();
}

void ACS::updateFlightControllerPIDloop()
{
  flightController.getDesiredState();

  // PID Controllers
  flightController.controlAngle(); // stabilize on angle setpoint
  // flightController.controlAngle2();         // stabilize on angle setpoint using cascaded method
  // flightController.controlRate();           // stabilize on rate setpoint

  // Actuator mixing and scaling
  flightController.controlMixer();
  flightController.scaleCommands();

  //Throttle cut check
  // flightController.throttleCut();           //directly sets motor commands to low based on state of ch5

  // Command Motors
  // flightController.commandMotors();         //sends command pulses to each motor pin using OneShot125 protocol

  // Command Servos
  flightController.commandServos();

  // Retreive Updated Radio Commands
  flightController.getCommands(receiver);
  // flightController.failSafe();

  // Update State Struct
  flightController.updateAircraftStateStruct();
}

void ACS::printDebugMessages()
{
  /** DEBUG FUNCTIONS **/
  // printRadioData();
  // printIMUdata();
  // printBMPdata();
  printDesiredState();
  // printRollPitchYaw();
  // printPIDoutput();
  // printMotorCommands();
  // printServoCommands();
  // printPIDvalues();
  // printAircraftState();
}

void ACS::printRadioData()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(flightController.channel_1_pwm);
    Serial.print(F(" CH2: "));
    Serial.print(flightController.channel_2_pwm);
    Serial.print(F(" CH3: "));
    Serial.print(flightController.channel_3_pwm);
    Serial.print(F(" CH4: "));
    Serial.print(flightController.channel_4_pwm);
    Serial.print(F(" CH5: "));
    Serial.print(flightController.channel_5_pwm);
    Serial.print(F(" CH6: "));
    Serial.print(flightController.channel_6_pwm);
    Serial.print(F(" CH7: "));
    Serial.println(flightController.channel_7_pwm);
  }
}

void ACS::printIMUdata()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();

    Serial.print(flightController.GyroX);
    Serial.print(F(","));
    Serial.print(flightController.GyroY);
    Serial.print(F(","));
    Serial.print(flightController.GyroZ);
    Serial.print(F(","));
    Serial.print(flightController.AccX);
    Serial.print(F(","));
    Serial.print(flightController.AccY);
    Serial.print(F(","));
    Serial.println(flightController.AccZ);
  }
}

void ACS::printBMPdata()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(flightController.internal_temp);
    Serial.print(",");
    Serial.print(flightController.pressure);
    Serial.print(",");
    Serial.println(flightController.altitude);
  }
}

void ACS::printDesiredState()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("thro_des: "));
    Serial.print(flightController.thro_des);
    Serial.print(F(" roll_des: "));
    Serial.print(flightController.roll_des);
    Serial.print(F(" pitch_des: "));
    Serial.print(flightController.pitch_des);
    Serial.print(F(" yaw_des: "));
    Serial.println(flightController.yaw_des);
  }
}

void ACS::printRollPitchYaw()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(flightController.roll_IMU);
    Serial.print(",");
    Serial.print(flightController.pitch_IMU);
    Serial.print(",");
    Serial.println(flightController.yaw_IMU);
  }
}

void ACS::printPIDoutput()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(flightController.roll_IMU);
    Serial.print(",");
    Serial.print(flightController.pitch_IMU);
    Serial.print(",");
    Serial.print(flightController.yaw_IMU);
    Serial.print(",");
    Serial.print(flightController.roll_PID);
    Serial.print(",");
    Serial.print(flightController.pitch_PID);
    Serial.print(",");
    Serial.print(flightController.yaw_PID);
    Serial.print(F(","));
    Serial.println(1 / flightController.dt);
  }
}

void ACS::printMotorCommands()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("m1_command: "));
    Serial.print(flightController.m1_command_PWM);
    Serial.print(F(" m2_command: "));
    Serial.print(flightController.m2_command_PWM);
    Serial.print(F(" m3_command: "));
    Serial.print(flightController.m3_command_PWM);
    Serial.print(F(" m4_command: "));
    Serial.print(flightController.m4_command_PWM);
    Serial.print(F(" m5_command: "));
    Serial.print(flightController.m5_command_PWM);
    Serial.print(F(" m6_command: "));
    Serial.println(flightController.m6_command_PWM);
  }
}

void ACS::printServoCommands()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(F("s1_command: "));
    Serial.print(flightController.s1_command_PWM);
    Serial.print(F(" s2_command: "));
    Serial.print(flightController.s2_command_PWM);
    Serial.print(F(" s3_command: "));
    Serial.print(flightController.s3_command_PWM);
    Serial.print(F(" s4_command: "));
    Serial.print(flightController.s4_command_PWM);
    Serial.print(F(" s5_command: "));
    Serial.print(flightController.s5_command_PWM);
    Serial.print(F(" s6_command: "));
    Serial.print(flightController.s6_command_PWM);
    Serial.print(F(" s7_command: "));
    Serial.println(flightController.s7_command_PWM);
  }
}

void ACS::printPIDvalues()
{
  // Roll - Pitch - Yaw
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();
    Serial.print(flightController.error_roll);
    Serial.print(F(","));
    Serial.print(flightController.error_pitch);
    Serial.print(F(","));
    Serial.print(flightController.error_yaw);
    Serial.print(F(","));
    Serial.print(flightController.integral_roll);
    Serial.print(F(","));
    Serial.print(flightController.integral_pitch);
    Serial.print(F(","));
    Serial.print(flightController.integral_yaw);
    Serial.print(F(","));
    Serial.print(flightController.derivative_roll);
    Serial.print(F(","));
    Serial.print(flightController.derivative_pitch);
    Serial.print(F(","));
    Serial.print(flightController.derivative_yaw);
    Serial.print(F(","));
    Serial.print(flightController.roll_PID);
    Serial.print(F(","));
    Serial.print(flightController.pitch_PID);
    Serial.print(F(","));
    Serial.println(flightController.yaw_PID);
  }
}

void ACS::printAircraftState()
{
  if (flightController.current_time - print_counter > 10000)
  {
    // Add mode values here
    Serial.print(flightController.stateFC.ned_accel_f.x);
    Serial.print(",");
    Serial.print(flightController.stateFC.ned_accel_f.y);
    Serial.print(",");
    Serial.print(flightController.stateFC.ned_accel_f.z);
    Serial.print(",");
    Serial.print(flightController.stateFC.body_gyro_f.p);
    Serial.print(",");
    Serial.print(flightController.stateFC.body_gyro_f.q);
    Serial.print(",");
    Serial.print(flightController.stateFC.body_gyro_f.r);
    Serial.print(",");
    Serial.print(flightController.stateFC.ned_speed_f.x);
    Serial.print(",");
    Serial.print(flightController.stateFC.ned_speed_f.y);
    Serial.print(",");
    Serial.print(flightController.stateFC.ned_speed_f.z);
    Serial.print(",");
    Serial.print(flightController.stateFC.quat_f.qi);
    Serial.print(",");
    Serial.print(flightController.stateFC.quat_f.qx);
    Serial.print(",");
    Serial.print(flightController.stateFC.quat_f.qy);
    Serial.print(",");
    Serial.print(flightController.stateFC.quat_f.qz);
    Serial.print(",");
    Serial.print(flightController.stateFC.eulers_f.theta);
    Serial.print(",");
    Serial.print(flightController.stateFC.eulers_f.psi);
    Serial.print(",");
    Serial.print(flightController.stateFC.eulers_f.phi);
    Serial.print(",");
    Serial.print(flightController.stateFC.lla_pos_f.lat);
    Serial.print(",");
    Serial.print(flightController.stateFC.lla_pos_f.lon);
    Serial.print(",");
    Serial.print(flightController.stateFC.lla_pos_f.alt);
    Serial.print(",");
    Serial.print(flightController.stateFC.actuator.throttle);
    Serial.print(",");
    Serial.print(flightController.stateFC.actuator.aileron);
    Serial.print(",");
    Serial.print(flightController.stateFC.actuator.elevator);
    Serial.print(",");
    Serial.print(flightController.stateFC.actuator.rudder);
    Serial.print(",");
    Serial.println(flightController.stateFC.actuator.flaps);
  }
}

/* Command-line Functions */
void ACS::execCallback(cmd *execmd)
{
  Command cmd(execmd);
  int argNum = cmd.countArgs(); // length of args

  enum execCommands
  {
    REBOOT = 1,     // Reboot Cmd
    CALIBRATE = 2,  // Calibrate Cmd
    SERVO_TEST = 3, // Servo Test
    FIND_POS = 4,   // Find Current Position
  };

  int curr_cmd = 0; // store found cmd
  bool verbose = 0;
  bool keep_log = 0;
  bool fast = 0;
  String fastArg; // dummy arg

  // Go through all arguments
  for (int i = 0; i < argNum; i++)
  {
    Argument arg = cmd.getArg(i);
    String argValue = arg.getValue();

    // Serial.printf("\targ: %s",argValue.c_str());
    // Serial.println();

    if (strcmp(argValue.c_str(), "reboot") == 0)
    {
      curr_cmd = REBOOT;
    }
    else if ((strcmp(argValue.c_str(), "calibrate") == 0) | (strcmp(argValue.c_str(), "calib") == 0))
    {
      curr_cmd = CALIBRATE;
    }
    else if ((strcmp(argValue.c_str(), "-verbose") == 0) | (strcmp(argValue.c_str(), "-v") == 0))
    {
      verbose = 1;
    }
    else if ((strcmp(argValue.c_str(), "-keep_logs") == 0) | (strcmp(argValue.c_str(), "-kl") == 0))
    {
      keep_log = 1;
    }
    else if ((strcmp(argValue.c_str(), "-fast") == 0) | (strcmp(argValue.c_str(), "-f") == 0))
    {
      fast = 1;
      //check for positional arg
      if (i + 1 < argNum)
      {
        fastArg = cmd.getArg(i + 1).getValue();
        Serial.printf("\t%s", fastArg.c_str());
        Serial.println();
      }
    }
  }

  switch (curr_cmd)
  {
  case REBOOT:
    Serial.println("INFO: Rebooting...");
    // this->setupBeep(4, 160, 70);
    delay(50);
    SCB_AIRCR = 0x05FA0004;
    break;
  case CALIBRATE:
    // this->flightController.calibrateAttitude(verbose);
    Serial.print("Executing Calibrating");
    break;
  default:
    break;
  }
}

void ACS::handleReceivedMessage(char *msg)
{
  String str(msg);

  Serial.printf("$ %s", msg);
  Serial.println("");

  this->cli.parse(str.c_str());

  // Check if a new error occurred
  if (this->cli.errored())
  {
    CommandError e = this->cli.getError();

    // Print the error, or do whatever you want with it
    // Serial.println(e.toString());
    Serial.printf("# %s\n", e.toString().c_str());
  }

  // First check if a newly parsed command is available
  if (this->cli.available())
  {
    // Serial.print("It appears to be a valid command!");

    // Get the command out of the queue
    Command cmd = this->cli.getCommand();

    // Get the Argument(s) you want
    Argument argSetting = cmd.getArgument("setting"); // via name
    Argument argValue = cmd.getArgument("value");     // via index
    // strcpy(setting, argSetting.getValue());
    // strcpy(value, argValue.getValue());
    String setting = argSetting.getValue();
    String value = argValue.getValue();

    // Check if it's the command you're looking for
    if (cmd == this->cmdGet)
    {
      processGetCommand(setting.c_str());
    }
    if (cmd == this->cmdSet)
    {
      processSetCommand(setting.c_str(), value.c_str());
      Serial.printf("# SetCmd: %s | Val: %s", setting.c_str());
      Serial.println();
    }
  }
}

void ACS::handleSerial()
{
  const int BUFF_SIZE = 100;         // make it big enough to hold your longest command
  static char buffer[BUFF_SIZE + 1]; // +1 allows space for the null terminator
  static int length = 0;             // number of characters currently in the buffer
                                     //    static boolean newData = false;

  //    if(Serial.available())
  // while (Serial.available() > 0 && newData == false) {
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if ((c == '\r') || (c == '\n'))
    {
      // if(c == ';') {
      // end-of-line received
      // Serial.println("end-of-line received");

      if (length > 0)
      {
        handleReceivedMessage(buffer);
        Serial.println();
      }
      length = 0;
      // newData = true;
    }
    else
    {
      if (length < BUFF_SIZE)
      {
        buffer[length++] = c; // append the received character to the array
        buffer[length] = 0;   // append the null terminator
                              // Serial.print("Caractere ajouté : "); Serial.println(c);
      }
      else
      {
        // buffer full - discard the received character
        Serial.println("# ERROR: buffer full");
      }
    }
  }
  // Serial.println("End of handleSerial() function...");
}

void ACS::processGetCommand(const char *setting)
{
  if (strcmp(setting, "buzzer") == 0)
  {
    int val = 0;
    Serial.printf("# key: %s, val: %i", setting, val);
    Serial.println();
  }
  else if (strcmp(setting, "state") == 0)
  {
    // cliAircraftState();
    Serial.println("# State Get Command");
  }
  else if ((strcmp(setting, "t_states") == 0) | (strcmp(setting, "t_state") == 0))
  {
    Serial.println("# [IDLE, TAXI, TAKEOFF, IN_FLIGHT, DESCENT, TOUCHDOWN, POWERDOWN]");
  }
  else
  {
    Serial.println("# Unrecognized `get <setting>` supplied");
  }
}

void ACS::processSetCommand(const char *setting, const char *value)
{
}

/**
 * @brief Destroy the ACS::ACS object
 */
ACS::~ACS()
{
}