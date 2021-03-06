#ifndef MCU_CONFIG_H
#define MCU_CONFIG_H

/**************************************************************/
/********************** System Config *************************/
/**************************************************************/
#define SERIAL_BAUD 115200 // USB Serial Data rate (bps)


/**************************************************************/
/*********************** System Defines ***********************/
/**************************************************************/
#define USE_PWM_RX // default
//#define USE_PPM_RX
//#define USE_SBUS_RX

#define USE_MPU6050_I2C
// #define USE_MPU9250_SPI // SPI to MPU-9250
// #define USE_MPU9250_I2C

#define USE_BMP280_I2C // Barometric Pressure Sensor

// #define GYRO_250DPS //default full scale range
//#define GYRO_500DPS
//#define GYRO_1000DPS
#define GYRO_2000DPS

#define ACCEL_2G //default full scale range
// #define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G

/**************************************************************/
/************* SERIAL PRINT-OUR DEFINES ***********************/
/**************************************************************/

/**
 * @brief quaternion components in a [w, x, y, z] format (not best for parsing
 * 
 */
// #define OUTPUT_READABLE_QUATERNION

/**
 * @brief (in degrees) calculated from the quaternions 
 * coming from the FIFO.
 */
// #define OUTPUT_READABLE_EULER

/**
 * @brief pitch/roll angles (in degrees) calculated from the 
 * quaternions coming from the FIFO. Note this also requires 
 * gravity vector calculations.
 */
// #define OUTPUT_READABLE_YAWPITCHROLL

/**
 * @brief 
 * components with gravity removed. This acceleration 
 * reference frame is not compensated for orientation, so +X 
 * is always +X according to thesensor, just without the 
 * effects of gravity.
 */
// #define OUTPUT_READABLE_REALACCEL

/**
 * @brief acceleration components with gravity removed and adjusted for 
 * the world frame of reference (yaw is relative to initial 
 * orientation, since no magnetometer is present in this 
 * case). Could be quite handy in some cases.
 */
// #define OUTPUT_READABLE_WORLDACCEL

/**
 * @brief format used for the InvenSense teapot demo
 * 
 */
//#define OUTPUT_TEAPOT

/**************************************************************/
/************* PIN ASSIGNMENT AND FAILSAFES *******************/
/**************************************************************/

// State Indicator Devices
// #define LED_BLUE 25
// #define LED_GREEN 33
// #define BUZZER_PIN 32

#define MPU9250_SS 10
#define BMP280_SS 4

#define INTERRUPT_PIN 2
#define LED_PIN LED_BUILTIN

// Actuator Configuration
#define MAX_SERVO_ANGLE 120
#define MIN_SERVO_ANGLE 60
#define MAX_SERVO_PWM 2100
#define MIN_SERVO_PWM 900

// GPS RECEIVER
#define BN220_TX 29
#define BN220_RX 28

// Receiver Configuration
//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
//Radio:
//Note: If using SBUS, connect to pin 21 (RX5)
#define CH1_PIN 0
#define CH2_PIN 1
// #define CH3_PIN 17
#define CH4_PIN 2
#define CH5_PIN 3
#define CH6_PIN 4
#define CH7_PIN 22
#define CH8_PIN 23
// #define PPM_PIN 15

// Motor Configuration
// #define M1_PIN 0
// #define M2_PIN 1
// #define M3_PIN 14
// #define M4_PIN 3
// #define M5_PIN 4
// #define M6_PIN 5

// Servo Configuration
#define SERVO1_PIN 8 //Top Pin
#define SERVO2_PIN 7
#define SERVO3_PIN 6
#define SERVO4_PIN 5 //Bottom Pin
#define SERVO5_PIN 10
#define SERVO6_PIN 11
#define SERVO7_PIN 12

// Failsafe Configuration
#define CHAN1_FS 1500
#define CHAN2_FS 1500
#define CHAN3_FS 1500
#define CHAN4_FS 1500
#define CHAN5_FS 1500
#define CHAN6_FS 1500
#define CHAN7_FS 1000
#define CHAN8_FS 1000

#endif