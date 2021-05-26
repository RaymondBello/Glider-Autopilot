#pragma once

#include "Config.h"
#include <Arduino.h>
#include <PWMServo.h>
#include <Radio.h>
#include <I2Cdev.h>
#include <Wire.h>
#include "tools/std.h"
#include "State.h"

#if defined USE_MPU6050_I2C
#include "MPU6050_6Axis_MotionApps20.h"
#else
#error No IMU Defined
#endif

#if defined USE_BMP280_I2C
#include <Adafruit_BMP280.h>

#endif

enum SetpointControl
{
    SETPOINT_RC_RECEIVER,
    SETPOINT_ACS,
};

struct SetpointACS
{
    unsigned long roll_pwm = 1500; // default values
    unsigned long pitch_pwm = 1500;
    unsigned long throttle_pwm = 1000;
    unsigned long yaw_pwm = 1500;

    unsigned long deg2pwm(float deg)
    {
        float pwm = 5;
        return pwm;
    }

    unsigned long percent2pwm(int percentage)
    {
        unsigned long pwm = 0;
        return pwm;
    }
};

struct Servos
{
    PWMServo servo1;
    PWMServo servo2;
    PWMServo servo3;
    PWMServo servo4;
    PWMServo servo5;
    PWMServo servo6;
    PWMServo servo7;
};

struct Motor
{
    bool motor_armed = false;
    int value_percent = 0;
    int value_pwm = 1000;
    float value_scaled;
    PWMServo motorPWM;
};

class FC
{
private:
    /* Servo Pins */
    const int servo1Pin = SERVO1_PIN;
    const int servo2Pin = SERVO2_PIN;
    const int servo3Pin = SERVO3_PIN;
    const int servo4Pin = SERVO4_PIN;
    const int servo5Pin = SERVO5_PIN;
    const int servo6Pin = SERVO6_PIN;
    const int servo7Pin = SERVO7_PIN;

    //Filter parameters - Defaults tuned for 2kHz loop rate
    float B_madgwick = 0.04; //madgwick filter parameter
    float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    float B_gyro = 0.17;     //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    float B_mag = 1.0;       //Magnetometer LP filter parameter

    

public:
//Controller parameters (take note of defaults before modifying!):
    float i_limit = 25.0;  //Integrator saturation level, mostly for safety (default 25.0)
    float maxRoll = 30.0;  //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
    float maxPitch = 30.0; //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
    float maxYaw = 160.0;  //Max yaw rate in deg/sec

    float Kp_roll_angle = 0.2;   //Roll P-gain - angle mode
    float Ki_roll_angle = 0.3;   //Roll I-gain - angle mode
    float Kd_roll_angle = 0.05;  //Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0. Use B_loop_roll)
    float B_loop_roll = 0.9;     //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
    float Kp_pitch_angle = 0.2;  //Pitch P-gain - angle mode
    float Ki_pitch_angle = 0.3;  //Pitch I-gain - angle mode
    float Kd_pitch_angle = 0.05; //Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0. Use B_loop_pitch)
    float B_loop_pitch = 0.9;    //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

    float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
    float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
    float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
    float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
    float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
    float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

    float Kp_yaw = 0.3;     //Yaw P-gain
    float Ki_yaw = 0.05;    //Yaw I-gain
    float Kd_yaw = 0.00015; //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
    
    unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm;
    unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev, channel_5_pwm_prev, channel_6_pwm_prev, channel_7_pwm_prev, channel_8_pwm_prev;
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    float MagX, MagY, MagZ;
    float AccX_prev, AccY_prev, AccZ_prev;
    float GyroX_prev, GyroY_prev, GyroZ_prev;
    float MagX_prev, MagY_prev, MagZ_prev;
    float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
    float roll_IMU, pitch_IMU, yaw_IMU;
    float roll_IMU_prev, pitch_IMU_prev;

    //Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
    float MagErrorX = 0.0;
    float MagErrorY = 0.0;
    float MagErrorZ = 0.0;
    float MagScaleX = 1.0;
    float MagScaleY = 1.0;
    float MagScaleZ = 1.0;
    //initialize quaternion for madgwick filter
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    //Barometric pressure sensor
    float internal_temp, pressure, altitude;

    uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
    bool dmpReady = false; // set true if DMP init was successful
    uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)
    float dt;
    unsigned long current_time, prev_time;
    unsigned long blink_counter, blink_delay;
    unsigned long beep_counter, beep_delay;
    bool blinkAlternate;
    bool beepAlternate;

    //Normalized desired state:
    float thro_des, roll_des, pitch_des, yaw_des;
    float roll_passthru, pitch_passthru, yaw_passthru;

    //Controller:
    float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
    float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
    float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

    //Mixer
    float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
    int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
    float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
    int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

#ifdef AIRFRAME_FIXEDWING
    Motor motorFC_1; // Flight Controller Main Motor
#endif
#ifdef AIRFRAME_QUADCOPTER
    Motor motor1;
    Motor motor2;
    Motor motor3;
    Motor motor4;
#endif

    State stateFC;        // Flight Controller State
    Servos actFC;         // Flight Controller Actuator
    MPU6050 IMU;          // Flight Controller Inertial Measurement Unit
    Adafruit_BMP280 Baro; // Flight Controller Barometer

    SetpointControl setpoint_ctrl;
    SetpointACS setpoint_acs;

    FC(/* args */);
    ~FC();
    void init();
    BoolInt initIMU();
    BoolInt initBaro();
    void calculate_imu_error();
    void get_imu_data();
    void get_baro_data();
    void calibrate_attitude(bool verbose);
    void madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
    void madgwick_6dof(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
    float inverse_sqrt(float x);
    void loop_rate(int freq);
    void loop_blink();
    void loop_beep();
    void get_desired_aircraft_state();
    void control_desired_angle();
    void control_mixer();
    void scale_commands();
    void cut_throttle();
    void command_motors();
    void command_servos();
    void get_commands(Radio receiver);
    void update_aircraft_state_struct();
    void send_heartbeat_msg();
};

FC::FC(/* args */)
{
}

FC::~FC()
{
}

void FC::init()
{
#ifdef AIRFRAME_FIXEDWING
    // Setup Actuators
    this->actFC.servo1.attach(servo1Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo2.attach(servo2Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo3.attach(servo3Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo4.attach(servo4Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo5.attach(servo5Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo6.attach(servo6Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo7.attach(servo7Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);

    // Initialize & Arm Motors
    // this->motorFC_1.actMotor.attach(servo1Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->motorFC_1.motorArmed = true;
    this->motorFC_1.throttlePercent = 0;

    //Arm Servos
    this->actFC.servo1.write(0);
    this->actFC.servo2.write(0);
    this->actFC.servo3.write(0);
    this->actFC.servo4.write(0);
    this->actFC.servo5.write(0);
    this->actFC.servo6.write(0);
    this->actFC.servo7.write(0);

    // Setup Channel failsafes
    this->channel_1_pwm = CHAN1_FS; //ailerons
    this->channel_2_pwm = CHAN2_FS; //elevator
    this->channel_4_pwm = CHAN3_FS; //throttle
    this->channel_5_pwm = CHAN4_FS; //rudd
    this->channel_6_pwm = CHAN5_FS; //left dial //gear, greater than 1500 = throttle cut
    this->channel_7_pwm = CHAN6_FS; //right dial
    this->channel_8_pwm = CHAN8_FS; //right 2-way

    this->setpoint_ctrl = SETPOINT_RC_RECEIVER;
#endif

#ifdef AIRFRAME_QUADCOPTER
    // Setup Actuators
    this->motor1.motor_armed = bool(this->motor1.motorPWM.attach(SERVO1_PIN, MIN_SERVO_PWM, MAX_SERVO_PWM));
    this->motor2.motor_armed = bool(this->motor2.motorPWM.attach(SERVO2_PIN, MIN_SERVO_PWM, MAX_SERVO_PWM));
    this->motor3.motor_armed = bool(this->motor3.motorPWM.attach(SERVO3_PIN, MIN_SERVO_PWM, MAX_SERVO_PWM));
    this->motor4.motor_armed = bool(this->motor4.motorPWM.attach(SERVO4_PIN, MIN_SERVO_PWM, MAX_SERVO_PWM));

    //Arm Servos
    this->motor1.motorPWM.write(0);
    this->motor2.motorPWM.write(0);
    this->motor3.motorPWM.write(0);
    this->motor4.motorPWM.write(0);

    // Setup Channel failsafes
    this->channel_1_pwm = CHAN1_FS; //ailerons
    this->channel_2_pwm = CHAN2_FS; //elevator
    this->channel_4_pwm = CHAN3_FS; //throttle
    this->channel_5_pwm = CHAN4_FS; //rudder
    this->channel_6_pwm = CHAN5_FS; //left dial //gear, greater than 1500 = throttle cut
    this->channel_7_pwm = CHAN6_FS; //right dial
    this->channel_8_pwm = CHAN8_FS; //right 2-way

    // this->setpoint_ctrl = SETPOINT_ACS;
    this->setpoint_ctrl = SETPOINT_RC_RECEIVER;
#endif
}

BoolInt FC::initIMU()
{
    BoolInt pass_err;

#if defined USE_MPU6050_I2C
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    // Wire.setClock(400000);       // 400kHz I2C clock. Comment this line if having compilation difficulties
    Serial.println("INFO: MPU6050 initialization...");
    this->IMU.initialize(GYRO_SCALE, ACCEL_SCALE);

    Serial.println("INFO: Initializing DMP...");
    pass_err.ErrCode = this->IMU.dmpInitialize();

    // Enter offsets here
    this->IMU.setXGyroOffset(220);
    this->IMU.setYGyroOffset(76);
    this->IMU.setZGyroOffset(-85);
    this->IMU.setZAccelOffset(1788); // 1688 factory default

    if (pass_err.ErrCode == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        this->IMU.CalibrateAccel(6);
        this->IMU.CalibrateGyro(6);
        this->IMU.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("\n\nINFO: Enabling DMP..."));
        this->IMU.setDMPEnabled(true);
        // Serial.print(F("Enabling interrupt detection (Teensy external interrupt "));
        // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        // Serial.println(F(""));
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = this->IMU.getIntStatus();
        Serial.println("INFO: DMP ready!");
        dmpReady = true;
        pass_err.flag = true;
        packetSize = this->IMU.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print("INFO: DMP Initialization failed (code ");
        Serial.print(pass_err.ErrCode);
        Serial.println(")");
    }

#elif defined USE_MPU9250_SPI
#endif

    return pass_err;
}

BoolInt FC::initBaro()
{
    BoolInt pass_err;

#if defined USE_BMP280_I2C
    // start communication to Barometer
    if (!Baro.begin())
    {
        Serial.println(F("Could not find BMP280 sensor, check wiring"));
        Serial.println("ERROR, Failed to connect to BMP280");
        pass_err.ErrCode = -1;
        pass_err.flag = false;
    }
    else
    {
        /* Default settings from datasheet. */
        Baro.setSampling(Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. */
                         Adafruit_BMP280::SAMPLING_X2,    /* Temp. oversampling */
                         Adafruit_BMP280::SAMPLING_X2,    /* Pressure oversampling */
                         Adafruit_BMP280::FILTER_X2,      /* Filtering. */
                         Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */
        Serial.println("INFO: Barometer Sampling Settings SET");
        pass_err.ErrCode = 0;
        pass_err.flag = true;
    }
#endif
    return pass_err;
}

void FC::calculate_imu_error()
{
    //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
    /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in get_imu_data(). This eliminates drift in the
   * measurement. 
   */
    // int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

    Serial.println("INFO: Calculating IMU error...");

    //Read IMU values 12000 times
    int c = 0;
    while (c < 12000)
    {
#if defined USE_MPU6050_I2C
        IMU.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#elif defined USE_MPU9250_SPI
        IMU.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
#endif

        AccX = AcX / ACCEL_SCALE_FACTOR;
        AccY = AcY / ACCEL_SCALE_FACTOR;
        AccZ = AcZ / ACCEL_SCALE_FACTOR;
        GyroX = GyX / GYRO_SCALE_FACTOR;
        GyroY = GyY / GYRO_SCALE_FACTOR;
        GyroZ = GyZ / GYRO_SCALE_FACTOR;

        //Sum all readings
        AccErrorX = AccErrorX + AccX;
        AccErrorY = AccErrorY + AccY;
        AccErrorZ = AccErrorZ + AccZ;
        GyroErrorX = GyroErrorX + GyroX;
        GyroErrorY = GyroErrorY + GyroY;
        GyroErrorZ = GyroErrorZ + GyroZ;
        c++;
    }
    //Divide the sum by 12000 to get the error value
    AccErrorX = AccErrorX / c;
    AccErrorY = AccErrorY / c;
    AccErrorZ = AccErrorZ / c - 1.0;
    GyroErrorX = GyroErrorX / c;
    GyroErrorY = GyroErrorY / c;
    GyroErrorZ = GyroErrorZ / c;
}

void FC::calibrate_attitude(bool verbose)
{
    //Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators assuming vehicle is powered up on level surface!

    Serial.println("INFO: Calibrating Attitude. Warming up madgwick filter...");
    //Warm up IMU and madgwick filter in simulated main loop

    int verbose_count = 0;

    for (int i = 0; i <= 10000; i++)
    {
        prev_time = current_time;
        current_time = micros();
        dt = (current_time - prev_time) / 1000000.0;
        get_imu_data();
        madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
        loop_rate(2000); //do not exceed 2000Hz

        if (verbose)
        {
            if (verbose_count == 100)
            {
                verbose_count = 0;
                Serial.printf("\tθ: %.2f, φ: %.2f, ψ: %.2f", pitch_IMU, roll_IMU, yaw_IMU);
                Serial.println();
            }
            verbose_count++;
        }
    }
    Serial.println("INFO: Calibration Complete\n");
}

void FC::get_imu_data()
{
    //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
    /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;

#if defined USE_MPU6050_I2C
    IMU.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#elif defined USE_MPU9250_SPI
    mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
#endif

    //Accelerometer
    AccX = AcX / ACCEL_SCALE_FACTOR; //G's
    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;
    //Correct the outputs with the calculated error values
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccZ = AccZ - AccErrorZ;
    //LP filter accelerometer data
    AccX = (1.0 - B_accel) * AccX_prev + B_accel * AccX;
    AccY = (1.0 - B_accel) * AccY_prev + B_accel * AccY;
    AccZ = (1.0 - B_accel) * AccZ_prev + B_accel * AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;

    //Gyro
    GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
    //LP filter gyro data
    GyroX = (1.0 - B_gyro) * GyroX_prev + B_gyro * GyroX;
    GyroY = (1.0 - B_gyro) * GyroY_prev + B_gyro * GyroY;
    GyroZ = (1.0 - B_gyro) * GyroZ_prev + B_gyro * GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;

    //Magnetometer
    MagX = MgX / 6.0; //uT
    MagY = MgY / 6.0;
    MagZ = MgZ / 6.0;
    //Correct the outputs with the calculated error values
    MagX = (MagX - MagErrorX) * MagScaleX;
    MagY = (MagY - MagErrorY) * MagScaleY;
    MagZ = (MagZ - MagErrorZ) * MagScaleZ;
    //LP filter magnetometer data
    MagX = (1.0 - B_mag) * MagX_prev + B_mag * MagX;
    MagY = (1.0 - B_mag) * MagY_prev + B_mag * MagY;
    MagZ = (1.0 - B_mag) * MagZ_prev + B_mag * MagZ;
    MagX_prev = MagX;
    MagY_prev = MagY;
    MagZ_prev = MagZ;
}

void FC::madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float mholder;

//use 6DOF algorithm if MPU6050 is being used
#if defined USE_MPU6050_I2C
    madgwick_6dof(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
#endif

    //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        madgwick_6dof(gx, gy, gz, ax, ay, az, invSampleFreq);
        return;
    }

    //Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    //Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        //Normalise accelerometer measurement
        recipNorm = inverse_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //Normalise magnetometer measurement
        recipNorm = inverse_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        //Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        //Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //Apply feedback step
        qDot1 -= B_madgwick * s0;
        qDot2 -= B_madgwick * s1;
        qDot3 -= B_madgwick * s2;
        qDot4 -= B_madgwick * s3;
    }

    //Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    //Normalise quaternion
    recipNorm = inverse_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //compute angles - NWU
    roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951; //degrees
    pitch_IMU = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;                //degrees
    yaw_IMU = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951; //degrees
}

void FC::madgwick_6dof(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq)
{
    //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
    /*
   * See description of madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    //Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    //Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        //Normalise accelerometer measurement
        recipNorm = inverse_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        //Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //Apply feedback step
        qDot1 -= B_madgwick * s0;
        qDot2 -= B_madgwick * s1;
        qDot3 -= B_madgwick * s2;
        qDot4 -= B_madgwick * s3;
    }

    //Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    //Normalise quaternion
    recipNorm = inverse_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //compute angles
    roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951; //degrees
    // pitch_IMU = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;                //degrees
    pitch_IMU = asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;
    yaw_IMU = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951; //degrees
}

float FC::inverse_sqrt(float x)
{
    //Fast inverse sqrt for madgwick filter
    /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
    //alternate form:
    unsigned int i = 0x5F1F1412 - (*(unsigned int *)&x >> 1);
    float tmp = *(float *)&i;
    float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
    return y;
}

void FC::loop_rate(int freq)
{
    //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
    float invFreq = 1.0 / freq * 1000000.0;
    unsigned long checker = micros();

    //Sit in loop until appropriate time has passed
    while (invFreq > (checker - current_time))
    {
        checker = micros();
    }
}

void FC::loop_blink()
{
    //DESCRIPTION: Blink LED on board to indicate main loop is running
    /*
   * It looks cool.
   */
    if (current_time - blink_counter > blink_delay)
    {
        blink_counter = micros();
        digitalWrite(13, blinkAlternate); //pin 13 is built in LED

        if (blinkAlternate == 1)
        {
            blinkAlternate = 0;
            blink_delay = 100000;
        }
        else if (blinkAlternate == 0)
        {
            blinkAlternate = 1;
            blink_delay = 2000000;
        }
    }
}

void FC::loop_beep()
{

    if (current_time - beep_counter > beep_delay)
    {
        beep_counter = micros();
        digitalWrite(BUZZER_PIN, beepAlternate); //pin 13 is built in LED

        if (beepAlternate == 1)
        {
            beepAlternate = 0;
            beep_delay = 500000;
        }
        else if (beepAlternate == 0)
        {
            beepAlternate = 1;
            beep_delay = 9000000;
        }
    }
}

void FC::send_heartbeat_msg()
{
}

void FC::get_desired_aircraft_state()
{
    /*
    Normalizes desired control values to appropriate values
    Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des.
    These are computed by using the raw RC pwm commands and scaling them to be within our limits defined in config.
    roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
    (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
    yaw_passthru variables, to be used in commanding motors/servos with direct un-stabilized commands in control_mixer().
    */

    // roll_des = (channel_1_pwm - 1500.0) / 500.0;  //between -1 and 1
    // pitch_des = (channel_2_pwm - 1500.0) / 500.0; //between -1 and 1
    // thro_des = (channel_3_pwm - 1000.0) / 1000.0; //between 0 and 1
    // yaw_des = (channel_4_pwm - 1500.0) / 500.0;   //between -1 and 1

    //Constrain within normalized bounds
    thro_des = constrain(thro_des, 0.0, 1.0);               //between 0 and 1
    roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;    //between -maxRoll and +maxRoll
    pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; //between -maxPitch and +maxPitch
    yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;       //between -maxYaw and +maxYaw

    roll_passthru = roll_des / (2 * maxRoll);
    pitch_passthru = pitch_des / (2 * maxPitch);
    yaw_passthru = yaw_des / (2 * maxYaw);
}

void FC::control_desired_angle()
{
    //Roll
    error_roll = roll_des - roll_IMU;
    integral_roll = integral_roll_prev + error_roll * dt;
    if (channel_3_pwm < 1060)
    {
        //don't let integrator build if throttle is too low
        integral_roll = 0;
    }
    integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
    derivative_roll = GyroX;
    roll_PID = (float)0.01 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll); //scaled by .01 to bring within -1 to 1 range

    //Pitch
    error_pitch = pitch_des - pitch_IMU;
    integral_pitch = integral_pitch_prev + error_pitch * dt;
    if (channel_3_pwm < 1060)
    { //don't let integrator build if throttle is too low
        integral_pitch = 0;
    }
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
    derivative_pitch = GyroY;
    pitch_PID = (float).01 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch); //scaled by .01 to bring within -1 to 1 range

    //Yaw, stabilize on rate from GyroZ
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw * dt;
    if (channel_3_pwm < 1060)
    { //don't let integrator build if throttle is too low
        integral_yaw = 0;
    }
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev) / dt;
    yaw_PID = (float).01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range

    //Update roll variables
    integral_roll_prev = integral_roll;
    //Update pitch variables
    integral_pitch_prev = integral_pitch;
    //Update yaw variables
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;
}

void FC::control_mixer()
{
    /*
   Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   normalized (0 to 1) thro_des command for throttle control. Can also apply direct un-stabilized commands from the with 
   roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables 
   are used in scale_commands() in preparation to be sent to the motor ESCs and servos.
   */
#if defined USE_DIFFERENTIAL_THRUST
    m1_command_scaled = thro_des + yaw_des;
    m2_command_scaled = thro_des - yaw_des;
#else
    m1_command_scaled = thro_des;
    m2_command_scaled = thro_des;
#endif

#if defined(AIRFRAME_FIXEDWING)
    //0.5 is centered servo, 0 is zero throttle if connecting to ESC for conventional PWM, 1 is max throttle
    s1_command_scaled = pitch_PID;
    s2_command_scaled = roll_PID;
    s3_command_scaled = yaw_PID;
    s4_command_scaled = 0;
    s5_command_scaled = 0;
    s6_command_scaled = 0;
    s7_command_scaled = 0;

#elif defined(AIRFRAME_QUADCOPTER)
    motor1.value_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;
    motor2.value_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;
    motor3.value_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;
    motor4.value_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;

    motor1.value_scaled = 0 + ((180 - 0) / (1.5 - 0)) * (motor1.value_scaled - 0);
    motor2.value_scaled = 0 + ((180 - 0) / (1.5 - 0)) * (motor2.value_scaled - 0);
    motor3.value_scaled = 0 + ((180 - 0) / (1.5 - 0)) * (motor3.value_scaled - 0);
    motor4.value_scaled = 0 + ((180 - 0) / (1.5 - 0)) * (motor4.value_scaled - 0);

    s1_command_scaled = motor1.value_scaled;
    s2_command_scaled = motor2.value_scaled;
    s3_command_scaled = motor3.value_scaled;
    s4_command_scaled = motor4.value_scaled;
#endif
    //Example use of the linear fader for float type variables. Linearly interpolate between minimum and maximum values for Kp_pitch_rate variable based on state of channel 6:
    // if (channel_6_pwm > 1500){ //go to max specified value in 5.5 seconds
    //     //parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
    //     Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 5.5, 1, 2000);
    // }
    // if (channel_6_pwm < 1500) { //go to min specified value in 2.5 seconds
    //     //parameter, minimum value, maximum value, fadeTime, state (0 min or 1 max), loop frequency
    //     Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 2.5, 0, 2000);
    // }
}

void FC::scale_commands()
{
    /*
   mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. 
   sX_command_scaled variables from the mixer function are scaled to 0-180 for the servo library using standard PWM.
   mX_command_PWM are updated here which are used to command the motors in command_motors(). 
   sX_command_PWM are updated which are used to command the servos.
   */

#if defined(AIRFRAME_FIXEDWING)
    //Scaled to 125us - 250us for oneshot125 protocol
    m1_command_PWM = m1_command_scaled * 125 + 125;
    m2_command_PWM = m2_command_scaled * 125 + 125;
    //Constrain commands to motors within oneshot125 bounds
    m1_command_PWM = constrain(m1_command_PWM, 125, 250);
    m2_command_PWM = constrain(m2_command_PWM, 125, 250);

    //Scaled to 0-180 for servo library
    s1_command_PWM = s1_command_scaled * 180;
    s2_command_PWM = s2_command_scaled * 180;
    s3_command_PWM = s3_command_scaled * 180;
    s4_command_PWM = s4_command_scaled * 180;
    s5_command_PWM = s5_command_scaled * 180;
    s6_command_PWM = s6_command_scaled * 180;
    s7_command_PWM = s7_command_scaled * 180;
    //Constrain commands to servos within servo library bounds
    s1_command_PWM = constrain(s1_command_PWM, 0, 180);
    s2_command_PWM = constrain(s2_command_PWM, 0, 180);
    s3_command_PWM = constrain(s3_command_PWM, 0, 180);
    s4_command_PWM = constrain(s4_command_PWM, 0, 180);
    s5_command_PWM = constrain(s5_command_PWM, 0, 180);
    s6_command_PWM = constrain(s6_command_PWM, 0, 180);
    s7_command_PWM = constrain(s7_command_PWM, 0, 180);
#elif defined(AIRFRAME_QUADCOPTER)

    //Scaled to 0-180 for servo library
    motor1.value_pwm = motor1.value_scaled * 2000;
    motor2.value_pwm = motor2.value_scaled * 2000;
    motor3.value_pwm = motor3.value_scaled * 2000;
    motor4.value_pwm = motor4.value_scaled * 2000;

    //Constrain commands to servos within servo library bounds
    motor1.value_pwm = constrain(motor1.value_pwm, 1000, 2000);
    motor2.value_pwm = constrain(motor2.value_pwm, 1000, 2000);
    motor3.value_pwm = constrain(motor3.value_pwm, 1000, 2000);
    motor4.value_pwm = constrain(motor4.value_pwm, 1000, 2000);

#endif
}

void FC::cut_throttle()
{
    /*
    Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
    minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function called before command_motors() is called so that the last thing checked is if the user is giving permission to 
    command the motors to anything other than minimum value. Safety first. 
    */
    // if (channel_6_pwm < 1500)
    // {
    //     m1_command_PWM = 120;
    //     m2_command_PWM = 120;

    //     //uncomment if using servo PWM variables to control motor ESCs
    //     //s1_command_PWM = 0;
    //     //s2_command_PWM = 0;
    //     s3_command_PWM = 0;
    //     //s4_command_PWM = 0;
    //     //s5_command_PWM = 0;
    //     //s6_command_PWM = 0;
    //     //s7_command_PWM = 0;
    // }
}

void FC::command_motors()
{
#if defined(AIRFRAME_QUADCOPTER)

    // Throttle Arm Check
    if (this->channel_6_pwm > 1700)
    {
        motor1.motorPWM.write(int(motor1.value_scaled));
        motor2.motorPWM.write(int(motor2.value_scaled));
        motor3.motorPWM.write(int(motor3.value_scaled));
        motor4.motorPWM.write(int(motor4.value_scaled));

        // motor1.motorPWM.write(40);
        // motor2.motorPWM.write(30);
        // motor3.motorPWM.write(30);
        // motor4.motorPWM.write(30);
    }
    else
    {
        motor1.motorPWM.write(0);
        motor2.motorPWM.write(0);
        motor3.motorPWM.write(0);
        motor4.motorPWM.write(0);
    }

#endif
}

void FC::command_servos()
{
#if defined(AIRFRAME_FIXEDWING)
    actFC.servo1.write(s1_command_PWM);
    actFC.servo2.write(s2_command_PWM);
    actFC.servo3.write(s3_command_PWM);
    actFC.servo4.write(s4_command_PWM);
// actFC.servo5.write(s5_command_PWM);
// actFC.servo6.write(s6_command_PWM);
// actFC.servo7.write(s7_command_PWM);
#endif
}

void FC::get_commands(Radio receiver)
{
    //DESCRIPTION: Get raw PWM values for every channel from the radio
    /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

#if defined(USE_PWM_RX) && defined(AIRFRAME_FIXEDWING)
    channel_1_pwm = receiver.getRadioPWM(1);
    channel_2_pwm = receiver.getRadioPWM(2);
    channel_3_pwm = receiver.getRadioPWM(3);
    channel_4_pwm = receiver.getRadioPWM(4);
    channel_5_pwm = receiver.getRadioPWM(5);
    channel_6_pwm = receiver.getRadioPWM(6);
    channel_7_pwm = receiver.getRadioPWM(7);

#elif defined(USE_PWM_RX) && defined(AIRFRAME_QUADCOPTER)
    switch (this->setpoint_ctrl)
    {
    case SETPOINT_RC_RECEIVER:
    {
        channel_1_pwm = receiver.getRadioPWM(1);
        channel_2_pwm = receiver.getRadioPWM(2);
        channel_3_pwm = receiver.getRadioPWM(3);
        channel_4_pwm = receiver.getRadioPWM(4);
        channel_5_pwm = receiver.getRadioPWM(5);
        channel_6_pwm = receiver.getRadioPWM(6);
        // Serial.println("Using RC");
        break;
    }
    case SETPOINT_ACS:
    {
        channel_1_pwm = setpoint_acs.roll_pwm;
        channel_2_pwm = setpoint_acs.pitch_pwm;
        channel_3_pwm = setpoint_acs.throttle_pwm;
        channel_4_pwm = setpoint_acs.yaw_pwm;
        // Serial.println("Using ACS");
        break;
    }
    default:
    {
        this->setpoint_ctrl = SETPOINT_RC_RECEIVER;
        break;
    }
    }

#elif defined(USE_SBUS_RX)
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
        //sBus scaling below is for Taranis-Plus and X4R-SB
        float scale = 0.615;
        float bias = 895.0;
        channel_1_pwm = sbusChannels[0] * scale + bias;
        channel_2_pwm = sbusChannels[1] * scale + bias;
        channel_3_pwm = sbusChannels[2] * scale + bias;
        channel_4_pwm = sbusChannels[3] * scale + bias;
        channel_5_pwm = sbusChannels[4] * scale + bias;
        channel_6_pwm = sbusChannels[5] * scale + bias;
    }
#endif

    //Low-pass the critical commands and update previous values
    float b = 0.2; //lower=slower, higher=noisier
    channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
    channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
    channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
    channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
    channel_5_pwm = (1.0 - b) * channel_5_pwm_prev + b * channel_5_pwm;
    channel_6_pwm = (1.0 - b) * channel_6_pwm_prev + b * channel_6_pwm;
    channel_7_pwm = (1.0 - b) * channel_7_pwm_prev + b * channel_7_pwm;

    channel_1_pwm_prev = channel_1_pwm;
    channel_2_pwm_prev = channel_2_pwm;
    channel_3_pwm_prev = channel_3_pwm;
    channel_4_pwm_prev = channel_4_pwm;
    channel_5_pwm_prev = channel_5_pwm;
    channel_6_pwm_prev = channel_6_pwm;
    channel_7_pwm_prev = channel_7_pwm;
}

void FC::update_aircraft_state_struct()
{
    FloatQuat aircraftQuaternion;
    NedCoor_f aircraftAcceleration;
    FloatRates aircraftAngularRate;
    FloatEulers aircraftEuler;
    // Actuator aircraftActuators;

    uint16_t zero_array[8] = {0};
    stateFC.setStateStatus(zero_array);

    aircraftQuaternion.qi = q0;
    aircraftQuaternion.qx = q1;
    aircraftQuaternion.qy = q2;
    aircraftQuaternion.qz = q3;
    stateFC.setStateQuaternion(&aircraftQuaternion);

    aircraftAcceleration.x = AccX;
    aircraftAcceleration.y = AccY;
    aircraftAcceleration.z = AccZ;
    stateFC.setStateAccelNED(&aircraftAcceleration);

    aircraftAngularRate.p = GyroX;
    aircraftAngularRate.q = GyroY;
    aircraftAngularRate.r = GyroZ;
    stateFC.setStateAngularRates(&aircraftAngularRate);

    aircraftEuler.theta = pitch_IMU;
    aircraftEuler.phi = roll_IMU;
    aircraftEuler.psi = yaw_IMU;
    stateFC.setStateEulers(&aircraftEuler);
}

void FC::get_baro_data()
{
#if defined USE_BMP280_I2C
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 250)
    {
        internal_temp = Baro.readTemperature();
        pressure = Baro.readPressure();
        altitude = Baro.readAltitude();
        prev_ms = millis();
    }
#endif
}
