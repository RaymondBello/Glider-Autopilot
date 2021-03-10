#ifndef _CONFIG_H_
#define _CONFIG_H_

/**************************************************************/
/********************** System Config *************************/
/**************************************************************/
#define SERIAL_BAUD 115200 // USB Serial Data rate (bps)

/**************************************************************/
/*********************** System Defines ***********************/
/**************************************************************/
#define USE_PWM_RX // default
// #define USE_PPM_RX
//#define USE_SBUS_RX

#define USE_MPU6050_I2C
// #define USE_MPU9250_SPI 
// #define USE_MPU9250_I2C_1

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
/************* PIN ASSIGNMENT AND FAILSAFES *******************/
/**************************************************************/

// State Indicator Devices
// #define LED_BLUE 25
// #define LED_GREEN 33
#define BUZZER_PIN 36

// #define MPU9250_SS 10
#define MPU_INTERRUPT_PIN 2
#define BMP280_SS 4

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
//NOTE: Pin 13 is reserved for onboard LED, 
//NOTE: Pins 18 and 19 are reserved for I2C in default setup

//Radio Receiver Channels
#define CH1_PIN 0
#define CH2_PIN 1
#define CH3_PIN 2
#define CH4_PIN 3
#define CH5_PIN 4
#define CH6_PIN 22
#define CH7_PIN 23
#define CH8_PIN 17
#define PPM_PIN 15

// Motor Configuration
// #define M1_PIN 0
// #define M2_PIN 1

// Servo Configuration
#define SERVO1_PIN 8
#define SERVO2_PIN 7
#define SERVO3_PIN 6
#define SERVO4_PIN 5
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

/**************************************************************/
/************* VEHICLE CONFIGURATION  *************************/
/**************************************************************/
// #define USE_DIFFERENTIAL_THRUST



#endif /* _CONFIG_H_ */