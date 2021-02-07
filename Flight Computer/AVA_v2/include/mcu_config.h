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

#define GYRO_250DPS //default full scale range
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

#define ACCEL_2G //default full scale range
// #define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G


/**************************************************************/
/************* PIN ASSIGNMENT AND FAILSAFES *******************/
/**************************************************************/
#define RXD2 16
#define TXD2 17

// State Indicator Devices
// #define LED_BLUE 25
// #define LED_GREEN 33
// #define BUZZER_PIN 32

#define MPU9250_SS 10
#define BMP280_SS 4

// Actuator Configuration
#define MAX_SERVO_ANGLE 120
#define MIN_SERVO_ANGLE 60
#define MAX_SERVO_PWM 2100
#define MIN_SERVO_PWM 900

// Receiver Configuration
//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
//Radio:
//Note: If using SBUS, connect to pin 21 (RX5)
#define CH1_PIN 15
#define CH2_PIN 16
#define CH3_PIN 17
#define CH4_PIN 20
#define CH5_PIN 21
#define CH6_PIN 22
#define PPM_PIN 23

// Motor Configuration
#define M1_PIN 0
#define M2_PIN 1
#define M3_PIN 2
#define M4_PIN 3
#define M5_PIN 4
#define M6_PIN 5

// Servo Configuration
#define SERVO1_PIN 6
#define SERVO2_PIN 7
#define SERVO3_PIN 8
#define SERVO4_PIN 9
#define SERVO5_PIN 10
#define SERVO6_PIN 11
#define SERVO7_PIN 12

// Failsafe Configuration
#define CHAN1_FS 1000
#define CHAN2_FS 1500
#define CHAN3_FS 1500
#define CHAN4_FS 1500
#define CHAN5_FS 2000
#define CHAN6_FS 2000

#endif