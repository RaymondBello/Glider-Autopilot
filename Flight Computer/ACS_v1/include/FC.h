#pragma once


#include <Config.h>
#include <Arduino.h>
#include <PWMServo.h>
#include <I2Cdev.h>
#include <Wire.h>
#include "config.h"
#include "tools/std.h"
#include "state.h"

#if defined USE_MPU6050_I2C
    #include "MPU6050_6Axis_MotionApps20.h"
#else
    #error No IMU Defined
#endif

#if defined USE_BMP280_I2C
    #include <Adafruit_BMP280.h>
    
#endif



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
    bool motorArmed = false;
    int throttlePercent;
    int throttlePWM;
    PWMServo actMotor;
};

class FC
{
private:
    /* data */
    const int servo1Pin = SERVO1_PIN;
    const int servo2Pin = SERVO2_PIN;
    const int servo3Pin = SERVO3_PIN;
    const int servo4Pin = SERVO4_PIN;
    const int servo5Pin = SERVO5_PIN;
    const int servo6Pin = SERVO6_PIN;
    const int servo7Pin = SERVO7_PIN;

public:
    unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm;
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    float MagX, MagY, MagZ;
    float AccX_prev, AccY_prev, AccZ_prev;
    float GyroX_prev, GyroY_prev, GyroZ_prev;
    float MagX_prev, MagY_prev, MagZ_prev;
    float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
    float roll_IMU, pitch_IMU, yaw_IMU;
    float roll_IMU_prev, pitch_IMU_prev;
    //Filter parameters - Defaults tuned for 2kHz loop rate
    float B_madgwick = 0.04; //Madgwick filter parameter
    float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    float B_gyro = 0.17;     //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    float B_mag = 1.0;       //Magnetometer LP filter parameter
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

    uint8_t mpuIntStatus;                           // holds actual interrupt status byte from MPU
    bool dmpReady = false;                          // set true if DMP init was successful
    uint16_t packetSize;                            // expected DMP packet size (default is 42 bytes)
    float dt;
    unsigned long current_time, prev_time;
    unsigned long print_counter, serial_counter;
    unsigned long blink_counter, blink_delay;
    unsigned long beep_counter, beep_delay;
    bool blinkAlternate;
    bool beepAlternate;

    State stateFC;                                  // Flight Controller State 
    Servos actFC;                                   // Flight Controller Actuator
    Motor motorFC_1;                                // Flight Controller Main Motor 
    MPU6050 IMU;                                    // Flight Controller Inertial Measurement Unit
    Adafruit_BMP280 Baro;                           // Flight Controller Barometer
    
    FC(/* args */);
    ~FC();
    void init();
    BoolInt initIMU();
    BoolInt initBaro();
    void calculateIMUerror();
    void getIMUdata();
    void calibrateAttitude(bool verbose);
    void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
    void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
    float invSqrt(float x);
    void loopRate(int freq);
    void loopBlink();
    void loopBeep();
};

FC::FC(/* args */)
{
}

FC::~FC()
{
}

void FC::init()
{

    // Setup Actuators
    this->actFC.servo1.attach(servo1Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo2.attach(servo2Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo3.attach(servo3Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo4.attach(servo4Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo5.attach(servo5Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo6.attach(servo6Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->actFC.servo7.attach(servo7Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);

    //Arm Servos
    this->actFC.servo1.write(90);
    this->actFC.servo2.write(90);
    this->actFC.servo3.write(90);
    this->actFC.servo4.write(90);
    this->actFC.servo5.write(0);
    this->actFC.servo6.write(0);
    this->actFC.servo7.write(0);

    // Setup Channel failsafes
    this->channel_1_pwm = CHAN1_FS; //ailerons
    this->channel_2_pwm = CHAN2_FS; //elevator
    this->channel_4_pwm = CHAN3_FS; //elev
    this->channel_5_pwm = CHAN4_FS; //rudd
    this->channel_6_pwm = CHAN5_FS; //left dial //gear, greater than 1500 = throttle cut
    this->channel_7_pwm = CHAN6_FS; //right dial
    this->channel_8_pwm = CHAN8_FS; //right 2-way


    // Initialize & Arm Motors
    // this->motorFC_1.actMotor.attach(servo1Pin, MIN_SERVO_PWM, MAX_SERVO_PWM);
    this->motorFC_1.motorArmed = true;
    this->motorFC_1.throttlePercent = 0;

}

BoolInt FC::initIMU()
{
    BoolInt pass_err;

    #if defined USE_MPU6050_I2C
        Wire.begin();
        Wire.setClock(1000000);         //Note this is 2.5 times the spec sheet 400 kHz max...
        // Wire.setClock(400000);       // 400kHz I2C clock. Comment this line if having compilation difficulties
        Serial.println("\tMPU6050 initialization...");
        this->IMU.initialize(GYRO_SCALE, ACCEL_SCALE);

        Serial.println("\tInitializing DMP...");
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
            Serial.println(F("\tEnabling DMP..."));
            this->IMU.setDMPEnabled(true);
            // Serial.print(F("Enabling interrupt detection (Teensy external interrupt "));
            // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
            // Serial.println(F(""));
            // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
            mpuIntStatus = this->IMU.getIntStatus();
            Serial.println("\tDMP ready!");
            dmpReady = true;
            pass_err.flag = true;
            packetSize = this->IMU.dmpGetFIFOPacketSize();
        }
        else 
        {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(pass_err.ErrCode);
            Serial.println(F(")"));
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
        Serial.println("[ERROR] : Failed to connect to BMP280");
        pass_err.ErrCode = -1;
        pass_err.flag = false;
    }
    else
    {
        /* Default settings from datasheet. */
        Baro.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X2,   /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X2,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */
        Serial.println("\tBarometer Sampling Settings SET");
        pass_err.ErrCode = 0;
        pass_err.flag = true;
    }
#endif
    return pass_err;
}

void FC::calculateIMUerror()
{
    //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
    /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
    // int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

    Serial.println("\tCalculating IMU error...");

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

void FC::calibrateAttitude(bool verbose)
{
    //Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators assuming vehicle is powered up on level surface!

    Serial.println("\tCalibrating Attitude. Warming up Madgwick filter...");
    //Warm up IMU and madgwick filter in simulated main loop

    int verbose_count = 0;

    for (int i = 0; i <= 10000; i++)
    {
        prev_time = current_time;
        current_time = micros();
        dt = (current_time - prev_time) / 1000000.0;
        getIMUdata();
        Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
        loopRate(2000); //do not exceed 2000Hz

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
    Serial.println("\tCalibration Complete\n");
}

void FC::getIMUdata()
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

void FC::Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float mholder;

//use 6DOF algorithm if MPU6050 is being used
#if defined USE_MPU6050_I2C
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
#endif

    //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
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
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
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
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //compute angles - NWU
    roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951; //degrees
    pitch_IMU = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;                //degrees
    yaw_IMU = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951; //degrees

}

void FC::Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq)
{
    //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
    /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
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
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
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
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
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
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
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

float FC::invSqrt(float x)
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

void FC::loopRate(int freq)
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

void FC::loopBlink()
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

void FC::loopBeep()
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