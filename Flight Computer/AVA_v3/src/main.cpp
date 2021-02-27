/*************************************************************************************************/
/**************************************** Libraries ***********************************************/
/*******************************************************************************/

#include <TinyGPS++.h>
#include <PWMServo.h>
#include <I2Cdev.h>
#include <Wire.h>

/*************************************************************************************************/
/************************************** User Defined Files ***************************************/
/*************************************************************************************************/
#include "config.h"
#include "tools/kalman_filter.h"
#include "tools/radio_comms.h"


/*************************************************************************************************/
/****************************************** Define Checks ***************************************/
/************************************************************************************************/
#if defined USE_MPU6050_I2C
    #include "MPU6050_6Axis_MotionApps20.h"
    MPU6050 mpu;
#elif defined USE_MPU9250_SPI
    #include <MPU9250.h>
    MPU9250 mpu9250(SPI, MPU9250_SS);
#elif defined USE_MPU9250_I2C_1
    #include <MPU9250.h>
    MPU9250 mpu9250(Wire,0x68);
#else
    #error No IMU Defined
#endif

#if defined USE_BMP280_I2C
    #include <Adafruit_BMP280.h>
    Adafruit_BMP280 bmp(&Wire);
#endif

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
    #define GYRO_FS_SEL_250 mpu9250.GYRO_RANGE_250DPS
    #define GYRO_FS_SEL_500 mpu9250.GYRO_RANGE_500DPS
    #define GYRO_FS_SEL_1000 mpu9250.GYRO_RANGE_1000DPS
    #define GYRO_FS_SEL_2000 mpu9250.GYRO_RANGE_2000DPS
    #define ACCEL_FS_SEL_2 mpu9250.ACCEL_RANGE_2G
    #define ACCEL_FS_SEL_4 mpu9250.ACCEL_RANGE_4G
    #define ACCEL_FS_SEL_8 mpu9250.ACCEL_RANGE_8G
    #define ACCEL_FS_SEL_16 mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
    #define GYRO_SCALE GYRO_FS_SEL_250
    #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
    #define GYRO_SCALE GYRO_FS_SEL_500
    #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
    #define GYRO_SCALE GYRO_FS_SEL_1000
    #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
    #define GYRO_SCALE GYRO_FS_SEL_2000
    #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
    #define ACCEL_SCALE ACCEL_FS_SEL_2
    #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
    #define ACCEL_SCALE ACCEL_FS_SEL_4
    #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
    #define ACCEL_SCALE ACCEL_FS_SEL_8
    #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
    #define ACCEL_SCALE ACCEL_FS_SEL_16
    #define ACCEL_SCALE_FACTOR 2048.0
#endif

/*************************************************************************************************************/
/*************************************** User Specified Variables *******************************************/
/***********************************************************************************************************/
//Radio failsafe values for every channel in the event that bad receiver data is detected. Recommended defaults:
unsigned long channel_1_fs = CHAN1_FS; //ailerons
unsigned long channel_2_fs = CHAN2_FS; //elevator
unsigned long channel_3_fs = CHAN3_FS; //elev
unsigned long channel_4_fs = CHAN4_FS; //rudd
unsigned long channel_5_fs = CHAN5_FS; //left dial //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = CHAN6_FS; //right dial
unsigned long channel_7_fs = CHAN7_FS; //right 3-way switch
unsigned long channel_8_fs = CHAN8_FS; //right 2-way switch

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


/************************************************************************************************************/
/******************************************* Variables ******************************************************/
/************************************************************************************************************/
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio comm:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm;

unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;

float q0 = 1.0f; //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'}; // packet structure for InvenSense teapot demo

//Barometric pressure sensor 
float internal_temp, pressure, altitude;

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


void mpu_interupt_update()
{

    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        if (mpuInterrupt && fifoCount < packetSize)
        {
            // try to get out of the infinite loop
            fifoCount = mpu.getFIFOCount();
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize)
    {
        /**
     * @brief Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
     * This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
     * 
     */
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT))
    {

        // read a packet from FIFO
        while (fifoCount >= packetSize)
        { // Lets catch up to NOW, someone is using the dreaded delay()!
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
        }
#ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print(",");
        Serial.print(q.x);
        Serial.print(",");
        Serial.print(q.y);
        Serial.print(",");
        Serial.print(q.z);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        // mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Serial.print("ypr\t");
        Serial.print(",");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print(",");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print(",");
        Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print(",");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print(",");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print(",");
        Serial.print(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print(",");
        Serial.print(aaReal.x);
        Serial.print(",");
        Serial.print(aaReal.y);
        Serial.print(",");
        Serial.print(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print(",");
        Serial.print(aaWorld.x);
        Serial.print(",");
        Serial.print(aaWorld.y);
        Serial.print(",");
        Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

float invSqrt(float x)
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

void dmpDataReady()
{
    mpuInterrupt = true;
}

void IMUinit()
{
    #if defined USE_MPU6050_I2C
        Wire.begin();
        Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
        // Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

        Serial.println("MPU9250 initialization...");
        mpu.initialize(GYRO_SCALE, ACCEL_SCALE);
        // pinMode(INTERRUPT_PIN, INPUT);

        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        // Enter offsets here
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788); // 1688 factory default

        // make sure it worked (returns 0 if so)
        if (devStatus == 0)
        {
            // Calibration Time: generate offsets and calibrate our MPU6050
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);

            // Serial.print(F("Enabling interrupt detection (Teensy external interrupt "));
            // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
            // Serial.println(F(""));
            // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

            mpuIntStatus = mpu.getIntStatus();

            Serial.println(F("DMP ready!"));
            dmpReady = true;

            packetSize = mpu.dmpGetFIFOPacketSize();
        }
        else
        {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        }

    #elif defined USE_MPU9250_SPI
        int status = mpu9250.begin();

        if (status < 0)
        {
            Serial.println("MPU9250 initialization unsuccessful");
            Serial.println("Check MPU9250 wiring or try cycling power");
            Serial.print("Status: ");
            Serial.println(status);
            while (1)
            {
            }
        }

        //From the reset state all registers should be 0x00, so we should be at
        //max sample rate with digital low pass filter(s) off.  All we need to
        //do is set the desired fullscale ranges
        mpu9250.setGyroRange(GYRO_SCALE);
        mpu9250.setAccelRange(ACCEL_SCALE);
        mpu9250.setMagCalX(MagErrorX, MagScaleX);
        mpu9250.setMagCalY(MagErrorY, MagScaleY);
        mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
        mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
    #endif
}

void getIMUdata()
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
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
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

void calculate_IMU_error()
{
    //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
    /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;

    Serial.println("Calculating IMU error...");

    //Read IMU values 12000 times
    int c = 0;
    while (c < 12000)
    {
#if defined USE_MPU6050_I2C
        mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#elif defined USE_MPU9250_SPI
        mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
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

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq)
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

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq)
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

void loopRate(int freq)
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

void calibrateAttitude()
{
    //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
    //Assuming vehicle is powered up on level surface!
    /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot. 
   */
    //Warm up IMU and madgwick filter in simulated main loop
    for (int i = 0; i <= 10000; i++)
    {
        prev_time = current_time;
        current_time = micros();
        dt = (current_time - prev_time) / 1000000.0;
        getIMUdata();
        Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
        loopRate(2000); //do not exceed 2000Hz
    }
}

void loopBlink()
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

void setupBlink(int numBlinks, int upTime, int downTime)
{
    for (int j = 1; j <= numBlinks; j++)
    {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}

void printIMUdata()
{
    if (current_time - print_counter > 10000)
    {
        print_counter = micros();
        
        Serial.print(GyroX);
        Serial.print(F(","));
        Serial.print(GyroY);
        Serial.print(F(","));
        Serial.print(GyroZ);
        Serial.print(F(","));
        Serial.print(AccX);
        Serial.print(F(","));
        Serial.print(AccY);
        Serial.print(F(","));
        Serial.println(AccZ);
    }
}

void printLoopRate()
{
    if (current_time - print_counter > 10000)
    {
        print_counter = micros();
        // Serial.print(F("dt = "));
        Serial.print(F(","));
        Serial.print(dt * 1000000.0);
    }
}

void failure_if_mpu_not_working()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        while (true)
        {
            Serial.println("MPU Initialization failed. Restart Teesny");
        }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2: "));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3: "));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4: "));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5: "));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6: "));
    Serial.println(channel_6_pwm);
  }
}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  #if defined USE_PPM_RX || defined USE_PWM_RX
    channel_1_pwm = getRadioPWM(1);
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_4_pwm = getRadioPWM(4);
    channel_5_pwm = getRadioPWM(5);
    channel_6_pwm = getRadioPWM(6);
    
  #elif defined USE_SBUS_RX
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.615;  
      float bias  = 895.0; 
      channel_1_pwm = sbusChannels[0] * scale + bias;
      channel_2_pwm = sbusChannels[1] * scale + bias;
      channel_3_pwm = sbusChannels[2] * scale + bias;
      channel_4_pwm = sbusChannels[3] * scale + bias;
      channel_5_pwm = sbusChannels[4] * scale + bias;
      channel_6_pwm = sbusChannels[5] * scale + bias; 
    }
  #endif
  
  //Low-pass the critical commands and update previous values
  float b = 0.2; //lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}


void setup()
{
    delay(1000);
    Serial.println("Booting Up");
    delay(4000);

    //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
    pinMode(LED_BUILTIN, OUTPUT); 
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(SERIAL_BAUD);

    //Initialize radio communication
    radioSetup();

    //Set radio channels to default (safe) values before entering main loop
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;

    Serial.println("Starting IMU setup");
    IMUinit();  //Initialize IMU communication

    delay(10);

    //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level
    calculate_IMU_error();

    delay(100);

    Serial.println("Calibrating Attitude. Warming up IMU & Madgwick filter");
    //Warm up the loop
    calibrateAttitude(); //helps to warm up IMU and Madgwick filter before finally entering main loop

    //Indicate entering main loop with 3 quick blinks
    setupBlink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)

    //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
    // calibrateMagnetometer(); //generates magnetometer error and scale factors
}

void loop()
{
    
    failure_if_mpu_not_working();

    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;

    printRadioData();

    loopBlink(); 
    
    // printIMUdata();      
    
    //Get vehicle state
    getIMUdata();
    getCommands();

    //Regulate loop rate
    loopRate(2000); 
}