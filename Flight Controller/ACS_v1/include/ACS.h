#pragma once

#include <Arduino.h>
#include <SimpleCLI.h>
#include "Config.h"
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

  bool mode1_complete;     // Flag to ensure init mode is executed once in loop()
  bool mode2_complete;

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
  void setup_mcu();
  void setup_radio_receiver() { receiver.radioSetup(); };
  void setup_led_blink(int numBlinks, int upTime, int downTime);
  void setup_beep(int numBeeps, int upTime, int downTime);
  BoolInt setup_flightcontroller();
  BoolInt calibrate_flightcontroller();
  void loop();
  void update_flightcontroller_time();
  void update_flightcontroller_orientation();
  void update_flightcontroller_pid_loop();
  void print_debug_msg();
  void print_radio_data();
  void print_imu_data();
  void print_baro_data();
  void print_desired_state();
  void print_roll_pitch_yaw();
  void print_pid_output();
  void print_motor_cmds();
  void print_servo_cmds();
  void print_pid_values();
  void print_serial_pkt();

  static void exec_cmd_callback(cmd *execmd);
  void handle_serial();
  void handle_received_msg(char *msg);
  void process_get_cmd(const char *setting);
  void process_set_cmd(const char *setting, const char *value);
};

/**
 * @brief Constructor
 * 
 */
ACS::ACS(/* args */)
{
  mode = Uninitialized;

  // Flags to ensure certain mode functions are executed once
  mode1_complete = false;
  mode2_complete = false;

  cmdGet = cli.addCommand("get");
  cmdGet.addPositionalArgument("setting");
  // cmdGet.addPositionalArgument("value");

  cmdSet = cli.addCommand("set");
  cmdSet.addPositionalArgument("setting");
  cmdSet.addPositionalArgument("val/ue");

  cmdExec = cli.addBoundlessCommand("exec", exec_cmd_callback);
}

void ACS::setup_mcu()
{
  this->stateACS.stateInit();

  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //Set Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
}

BoolInt ACS::setup_flightcontroller()
{

  this->flightController.init();
  BoolInt passErr = this->flightController.initIMU();

  if (passErr.flag)
  {
    passErr = this->flightController.initBaro();

    // Step all motors to zero
    flightController.motor1.motorPWM.write(0);
    flightController.motor2.motorPWM.write(0);
    flightController.motor3.motorPWM.write(0);
    flightController.motor4.motorPWM.write(0);
  }
  return passErr;
}

BoolInt ACS::calibrate_flightcontroller()
{
  BoolInt passErr;
  this->flightController.calculate_imu_error();
  delay(10);
  this->flightController.calibrate_attitude(0);
  delay(10);
  //Indicate entering main loop with 3 quick blinks
  setup_led_blink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)
  setup_beep(2, 160, 70);

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
      while (!mode1_complete){
        delay(1000);
        Serial.println("Uninitialized");
        this->setup_mcu();
        // this->mode = Mode::Initialization;

        Serial.println("INFO: UNINITIALIZED complete");
        mode1_complete = true;
      }
      handle_serial(); 
      break;
    }
    case Initialization:
    {
      while (!mode2_complete)
      {
        Serial.println("INFO: ACS-Mode: Initialization");
        this->setup_radio_receiver();
        
        BoolInt boolint = this->setup_flightcontroller();
        boolint.flag ? (0) : (this->mode = Mode::Error); // If setup fails, restart FC

        BoolInt boolint1 = this->calibrate_flightcontroller();
        boolint1.flag ? (0) : (this->mode = Mode::Error);

        // this->mode = Mode::Active;
        Serial.println("INFO: INITIALIZATION complete");
        mode2_complete = true;
      }
      handle_serial();
      break;
    }
    case Active:
    {
      // Serial.println("Active");
      flightController.loop_blink();
      update_flightcontroller_time();
      update_flightcontroller_orientation();
      update_flightcontroller_pid_loop();
      print_debug_msg();

      /** Handle CLI message **/
      // Serial.println("Wrote to Motors");

      handle_serial();

      flightController.loop_rate(2000);
      break;
    }
    case Idle:
    {
      Serial.println("INFO: ACS-Mode: Idle");
      break;
    }
    case Error:
    {
      Serial.println("INFO: ACS-Mode: Error");
      flightController.loop_beep();
      break;
    }
    default:
    {
      Serial.println("FATAL, FATAL ACS-Mode Error! Rebooting...");
      this->setup_beep(4, 160, 70);
      delay(50);
      SCB_AIRCR = 0x05FA0004; // Force Restart
      break;
    }
  }
}

void ACS::setup_led_blink(int numBlinks, int upTime, int downTime)
{
  for (int j = 1; j <= numBlinks; j++)
  {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void ACS::setup_beep(int numBeeps, int upTime, int downTime)
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

void ACS::update_flightcontroller_time()
{
  flightController.prev_time = flightController.current_time;
  flightController.current_time = micros();
  flightController.dt = (flightController.current_time - flightController.prev_time) / 1000000.0;
}

void ACS::update_flightcontroller_orientation()
{
  flightController.get_imu_data();
  flightController.madgwick(flightController.GyroX, -flightController.GyroY, -flightController.GyroZ, -flightController.AccX, flightController.AccY, flightController.AccZ, flightController.MagY, -flightController.MagX, flightController.MagZ, flightController.dt);
  flightController.get_baro_data();
}

void ACS::update_flightcontroller_pid_loop()
{
  flightController.get_desired_aircraft_state();

  // PID Controllers
  flightController.control_desired_angle(); // stabilize on angle setpoint
  // flightController.control_desired_angle2();         // stabilize on angle setpoint using cascaded method
  // flightController.controlRate();           // stabilize on rate setpoint

  // Actuator mixing and scaling
  flightController.control_mixer();
  flightController.scale_commands();

  //Throttle cut check
  // flightController.cut_throttle();           //directly sets motor commands to low based on state of ch5

  // Command Motors
  flightController.command_motors(); //sends command pulses to each motor pin using OneShot125 protocol

  // Command Servos
  flightController.command_servos();

  // Retreive Updated Radio Commands
  flightController.get_commands(receiver);
  // flightController.failSafe();

  // Update State Struct
  flightController.update_aircraft_state_struct();
}

void ACS::print_debug_msg()
{
  /** DEBUG FUNCTIONS **/
  // print_radio_data();
  // print_imu_data();
  // print_baro_data();
  // print_desired_state();
  // print_roll_pitch_yaw();
  // print_pid_output();
  // print_motor_cmds();
  // print_servo_cmds();
  // print_pid_values();
  // print_serial_pkt();
}

void ACS::print_radio_data()
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

void ACS::print_imu_data()
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

void ACS::print_baro_data()
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

void ACS::print_desired_state()
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

void ACS::print_roll_pitch_yaw()
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

void ACS::print_pid_output()
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

void ACS::print_motor_cmds()
{
  if (flightController.current_time - print_counter > 10000)
  {
    print_counter = micros();

#if defined(AIRFRAME_QUADCOPTER)
    Serial.print(F("m1_command: "));
    Serial.print(flightController.motor1.value_scaled);
    Serial.print(F(" m2_command: "));
    Serial.print(flightController.motor2.value_scaled);
    Serial.print(F(" m3_command: "));
    Serial.print(flightController.motor3.value_scaled);
    Serial.print(F(" m4_command: "));
    Serial.println(flightController.motor4.value_scaled);

// Serial.print(F("m1_command: "));
// Serial.print(flightController.motor1.value_pwm);
// Serial.print(F(" m2_command: "));
// Serial.print(flightController.motor2.value_pwm);
// Serial.print(F(" m3_command: "));
// Serial.print(flightController.motor3.value_pwm);
// Serial.print(F(" m4_command: "));
// Serial.println(flightController.motor4.value_pwm);
#endif

#if defined(AIRFRAME_FIXEDWING)
#endif
  }
}

void ACS::print_servo_cmds()
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

void ACS::print_pid_values()
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

void ACS::print_serial_pkt()
{
  if (flightController.current_time - print_counter > 10000)
  {
// Add mode values here
#ifdef AIRFRAME_FIXEDWING
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
#endif

#ifdef AIRFRAME_QUADCOPTER
    Serial.print(0); // Packet Type Indicator
    Serial.print(",");
    Serial.print(this->mode); // ACS Mode
    Serial.print(",");
    Serial.print(flightController.AccX);
    Serial.print(",");
    Serial.print(flightController.AccY);
    Serial.print(",");
    Serial.print(flightController.AccZ);
    Serial.print(",");
    Serial.print(flightController.GyroX);
    Serial.print(",");
    Serial.print(flightController.GyroY);
    Serial.print(",");
    Serial.print(flightController.GyroZ);
    Serial.print(",");
    Serial.print(flightController.q0);
    Serial.print(",");
    Serial.print(flightController.q1);
    Serial.print(",");
    Serial.print(flightController.q2);
    Serial.print(",");
    Serial.print(flightController.q3);
    Serial.print(",");
    Serial.print(flightController.pitch_IMU);
    Serial.print(",");
    Serial.print(flightController.roll_IMU);
    Serial.print(",");
    Serial.print(flightController.GyroZ);
    Serial.print(",");
    Serial.print(flightController.pitch_des);
    Serial.print(",");
    Serial.print(flightController.roll_des);
    Serial.print(",");
    Serial.print(flightController.yaw_des);
    Serial.print(",");
    Serial.print(flightController.error_pitch);
    Serial.print(",");
    Serial.print(flightController.error_roll);
    Serial.print(",");
    Serial.print(flightController.error_yaw);
    Serial.print(",");
    Serial.print(flightController.integral_pitch);
    Serial.print(",");
    Serial.print(flightController.integral_roll);
    Serial.print(",");
    Serial.print(flightController.integral_yaw);
    Serial.print(",");
    Serial.print(flightController.derivative_pitch);
    Serial.print(",");
    Serial.print(flightController.derivative_roll);
    Serial.print(",");
    Serial.print(flightController.derivative_yaw);
    Serial.print(",");
    Serial.print(flightController.pitch_PID);
    Serial.print(",");
    Serial.print(flightController.roll_PID);
    Serial.print(",");
    Serial.print(flightController.yaw_PID);
    Serial.print(",");
    Serial.print(flightController.motor1.value_scaled);
    Serial.print(",");
    Serial.print(flightController.motor2.value_scaled);
    Serial.print(",");
    Serial.print(flightController.motor3.value_scaled);
    Serial.print(",");
    Serial.print(flightController.motor4.value_scaled);
    Serial.print(",");
    Serial.print(flightController.motor1.value_pwm);
    Serial.print(",");
    Serial.print(flightController.motor2.value_pwm);
    Serial.print(",");
    Serial.print(flightController.motor3.value_pwm);
    Serial.print(",");
    Serial.print(flightController.motor4.value_pwm);
    Serial.print(",");
    Serial.print(flightController.setpoint_acs.throttle_pwm);
    Serial.print(",");
    Serial.print(flightController.setpoint_acs.pitch_pwm);
    Serial.print(",");
    Serial.print(flightController.setpoint_acs.roll_pwm);
    Serial.print(",");
    Serial.print(flightController.setpoint_acs.yaw_pwm);
    Serial.print(",");
    Serial.println(flightController.setpoint_ctrl);

#endif
  }
}

/* Command-line Functions */
void ACS::exec_cmd_callback(cmd *execmd)
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
    // this->setup_beep(4, 160, 70);
    delay(50);
    SCB_AIRCR = 0x05FA0004;
    break;
  case CALIBRATE:
    // this->flightController.calibrate_attitude(verbose);
    Serial.print("Executing Calibrating");
    break;
  default:
    break;
  }
}

void ACS::handle_received_msg(char *msg)
{
  String str(msg);

  #ifdef ECHO_CMD
    Serial.printf("$ %s", msg);
    Serial.println("");
  #endif

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
      process_get_cmd(setting.c_str());
    }
    if (cmd == this->cmdSet)
    {
      process_set_cmd(setting.c_str(), value.c_str());
    }
  }
}

void ACS::handle_serial()
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
        handle_received_msg(buffer);
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
  // Serial.println("End of handle_serial() function...");
}

void ACS::process_get_cmd(const char *setting)
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
    // Serial.print(flightController.setpoint_acs.throttle_pwm);
    // Serial.print(", ");
    // Serial.print(flightController.setpoint_acs.roll_pwm);
    // Serial.print(", ");
    // Serial.print(flightController.setpoint_acs.pitch_pwm);
    // Serial.print(", ");
    // Serial.println(flightController.setpoint_acs.yaw_pwm);

    Serial.print(flightController.motor1.value_pwm);
    Serial.print(", ");
    Serial.print(flightController.motor2.value_pwm);
    Serial.print(", ");
    Serial.print(flightController.motor3.value_pwm);
    Serial.print(", ");
    Serial.println(flightController.motor4.value_pwm);
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

int map_value(std::pair<int, int> inputRange, std::pair<int, int> outputRange, int value)
{
  int inValNorm = value - inputRange.first;
  int aUpperNorm = inputRange.second - inputRange.first;
  int normPosition = inValNorm / aUpperNorm;
  int bUpperNorm = outputRange.second - outputRange.first;
  int bValNorm = normPosition * bUpperNorm;
  int outVal = outputRange.first + bValNorm;
  return outVal;
}

void ACS::process_set_cmd(const char *setting, const char *value)
{
  // Initialize values for value bounding
  std::pair<int, int> outBoundPwm(1000, 2000);

  if (strcmp(setting, "mode") == 0)
  {

    Mode selected_mode = (Mode)atoi(value);

    switch (selected_mode)
    {
      case Uninitialized:
        if (this->mode == Uninitialized){
          Serial.println("ERROR: Already in UNINITIALIZED");
        }else {
          Serial.println("INFO: Setting mode to -> UNINITIALIZED");
          this->mode = Mode::Uninitialized;
          mode1_complete = false;  // Reset flag to only init once
        }
        break;
      
      case Initialization:
        if (this->mode == Initialization){
          Serial.println("ERROR: Already in INITIALIZATION");
        }else {
          Serial.println("INFO: Setting mode to -> INITIALIZATION");
          this->mode = Mode::Initialization;
          mode2_complete = false;  // Reset flag to only init once
        }
        break;

      case Active:
        if (this->mode == Active){
          Serial.println("ERROR: Already in ACTIVE");
        }else {
          Serial.println("INFO: Setting mode to -> ACTIVE");
          this->mode = Mode::Active;
        }
        break;
    }

  }

  else if (strcmp(setting, "ctrl") == 0)
  {

    switch (atoi(value))
    {
    case SETPOINT_RC_RECEIVER:
    {
      flightController.setpoint_ctrl = SETPOINT_RC_RECEIVER;
      Serial.print("# RC Receiver Mode: ");
      Serial.println(atoi(value));
      break;
    }
    case SETPOINT_ACS:
    {
      flightController.setpoint_ctrl = SETPOINT_ACS;
      Serial.print("# ACS Mode: ");
      Serial.println(atoi(value));
      break;
    }
    default:
    {
      Serial.print("# ERROR: can only be 0, 1. Received: ");
      Serial.println(atoi(value));
      break;
    }
    }
  }

  else if (strcmp(setting, "zero") == 0)
  {
    if (atoi(value) == 0)
    {
      flightController.setpoint_acs.roll_pwm = 1000;
      flightController.setpoint_acs.pitch_pwm = 1000;
      flightController.setpoint_acs.throttle_pwm = 1000;
      flightController.setpoint_acs.yaw_pwm = 1000;
      Serial.print("# All setpoints set to: ");
      Serial.println(atoi(value));
    }
    else
    {
      Serial.print("# 'all_zero' value must be 0 to confirm. value = ");
      Serial.println(atoi(value));
    }
  }

  else if (strcmp(setting, "thro") == 0)
  {
    Serial.print("# Throttle = ");
    //Simplified normalization between 0->100 => 1000->2000
    int throVal = 1000 + atoi(value) * 10;
    flightController.setpoint_acs.throttle_pwm = constrain(throVal, 1000, 2000);
    Serial.println(flightController.setpoint_acs.throttle_pwm);
  }
  else if (strcmp(setting, "roll") == 0)
  {
    Serial.print("# Roll = ");
    //Simplified normalization between -30->30 => 1000->2000
    int rollVal = 1500 + atoi(value) * (50 / 3);
    flightController.setpoint_acs.roll_pwm = constrain(rollVal, 1000, 2000);
    Serial.println(flightController.setpoint_acs.roll_pwm);
  }
  else if (strcmp(setting, "pitch") == 0)
  {
    Serial.print("# Pitch = ");
    //Simplified normalization between -30->30 => 1000->2000
    int pitchVal = 1500 + atoi(value) * (50 / 3);
    flightController.setpoint_acs.pitch_pwm = constrain(pitchVal, 1000, 2000);
    Serial.println(flightController.setpoint_acs.pitch_pwm);
  }
  else if (strcmp(setting, "yaw") == 0)
  {
    // initialize the input and output bounds
    std::pair<int, int> inBoundYaw(0, 360);
    // flightController.setpoint_acs.yaw_pwm = map_value(inBoundYaw, outBoundPwm, atoi(value));
    Serial.print("# Yaw = ");
    Serial.println(map_value(inBoundYaw, outBoundPwm, atoi(value)));
  }
}

/**
 * @brief Destroy the ACS::ACS object
 */
ACS::~ACS()
{
}