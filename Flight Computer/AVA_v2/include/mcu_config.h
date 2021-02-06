#ifdef MCU_CONFIG_H
#define MCU_CONFIG_H

/*********************************/
/******* System Config ***********/
/*********************************/
#define SERIAL_BAUD 115200


/*********************************/
/******* PIN ASSIGNMENT ***********/
/*********************************/
#define RXD2 16
#define TXD2 17

// #define HSPI_MISO 27
// #define HSPI_MOSI 13
// #define HSPI_SCLK 14
// #define HSPI_SS 15

// State Indicator Devices
#define LED_BLUE 25
#define LED_GREEN 33
#define BUZZER_PIN 32

#define IMU_CS 5
#define BARO_CS 4

// Actuator Configuration
#define L_W_SERVO 13
#define R_W_SERVO 12

#define MAX_SERVO_ANGLE 120
#define MIN_SERVO_ANGLE 60
#define MAX_SERVO_PWM 2000
#define MIN_SERVO_PWM 1000

// Receiver Configuration
#define RX_CHANNEL_1 14
#define RX_CHANNEL_2 27
#define RX_CHANNEL_3 22

#endif