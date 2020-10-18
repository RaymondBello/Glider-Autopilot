
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>

#define LED_RED D5
#define LED_GREEN D6
#define LED_BLUE D7

#define GPSBaud 9600               //GPS Baud rate
#define Serial_Monitor_Baud 115200 

// #define RX_PIN D4                  // White Wire
// #define TX_PIN D3                  // Green Wire

// Global Constants
#define MPU_ADDRESS 0x68

const int MPU_addr = MPU_ADDRESS; // i2c address of the MPU-6050
float AccX, AccY, AccZ, Tmp;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// State Constants
const int CURRENT_STATE = 0;
int STATE_POOL[7] = {0, 1, 2, 3, 4, 5, 6};

// Global variables to print to serial monitor at a steady rate
unsigned long lastPrint = 0;
const unsigned int PRINT_RATE = 100;

unsigned int accelReadCounter = 0;
unsigned int gyroReadCounter = 0;
unsigned int magReadCounter = 0;
unsigned int tempReadCounter = 0;

const char *ssid = "baca";
const char *password = "randy053";

WebSocketsServer webSocket = WebSocketsServer(80);

// Kalman filter function header
float KALMAN(float U);

char buffer_udp_out[100];
char buffer_udp_in[100];

// Called when receiving websocket packet
void onWebSocketEvent(u_int num, WStype_t type, uint8_t *payload, size_t length)
{

    // Determine type of Websocket event
    switch (type)
    {
    case (WStype_DISCONNECTED):
    {
        Serial.printf("[%u] Disconnected.\n", num);
        break;
    }

    case (WStype_CONNECTED):
    {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from ", num);
        Serial.println(ip.toString());
        break;
    }

    case (WStype_TEXT):
    {
        sprintf(buffer_udp_in, "%s", payload);
        Serial.println(buffer_udp_in);

        // Serial.printf("[%u]: %s\n", num, payload);

        sprintf(buffer_udp_out, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", AccX, AccY, AccZ, Tmp, GyroX, GyroY, GyroZ);
        webSocket.sendTXT(num, buffer_udp_out);
        break;
    }

    case (WStype_BIN):
    case (WStype_ERROR):
    case (WStype_FRAGMENT_TEXT_START):
    case (WStype_FRAGMENT):
    case (WStype_FRAGMENT_FIN):
    default:
        break;
    }
}

void i2c_setup()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x00); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
}

void calculate_IMU_error()
{
    // We can call this function in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    // Read accelerometer values 200 times
    while (c < 200)
    {
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 6);
        AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
        AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
        AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
        // Sum all readings
        AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
        AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
        c++;
    }
    //Divide the sum by 200 to get the error value
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    c = 0;
    // Read gyro values 200 times
    while (c < 200)
    {
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 6);
        GyroX = Wire.read() << 8 | Wire.read();
        GyroY = Wire.read() << 8 | Wire.read();
        GyroZ = Wire.read() << 8 | Wire.read();
        // Sum all readings
        GyroErrorX = GyroErrorX + (GyroX / 131.0);
        GyroErrorY = GyroErrorY + (GyroY / 131.0);
        GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
        c++;
    }
    //Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
    // Print the error values on the Serial Monitor
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY);
    Serial.print("GyroErrorX: ");
    Serial.println(GyroErrorX);
    Serial.print("GyroErrorY: ");
    Serial.println(GyroErrorY);
    Serial.print("GyroErrorZ: ");
    Serial.println(GyroErrorZ);
}

void i2c_initialize()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
}

float KALMAN_AccX(float U)
{
    // U = Noisy Measurement, returns U_hat = Filter Estimate

    //constants (static)
    static const float R = 40;   // Noise covariance (its actually 10)
    static const float H = 1.00; // Measurement map scalar
    static float Q = 10;         //initial estimated covariance
    static float P = 0;          //initial error covariance (must be 0)
    static float U_hat = 0;      // initial estimated state (assume we dont know)
    static float K = 0;          // Kalman Gain

    //begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    //Update error covariance
    P = (1 - K * H) * P + Q;

    // Return the estimate of U_hat
    return U_hat;
}

float KALMAN_AccY(float U)
{
    // U = Noisy Measurement, returns U_hat = Filter Estimate

    //constants (static)
    static const double R = 40;   // Noise covariance (its actually 10)
    static const double H = 1.00; // Measurement map scalar
    static double Q = 10;         //initial estimated covariance
    static double P = 0;          //initial error covariance (must be 0)
    static double U_hat = 0;      // initial estimated state (assume we dont know)
    static double K = 0;          // Kalman Gain

    //begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    //Update error covariance
    P = (1 - K * H) * P + Q;

    // Return the estimate of U_hat
    return U_hat;
}

float KALMAN_AccZ(float U)
{
    // U = Noisy Measurement, returns U_hat = Filter Estimate

    //constants (static)
    static const double R = 40;   // Noise covariance (its actually 10)
    static const double H = 1.00; // Measurement map scalar
    static double Q = 10;         //initial estimated covariance
    static double P = 0;          //initial error covariance (must be 0)
    static double U_hat = 0;      // initial estimated state (assume we dont know)
    static double K = 0;          // Kalman Gain

    //begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    //Update error covariance
    P = (1 - K * H) * P + Q;

    // Return the estimate of U_hat
    return U_hat;
}

float KALMAN_GyroX(float U)
{
    // U = Noisy Measurement, returns U_hat = Filter Estimate

    //constants (static)
    static const double R = 40;   // Noise covariance (its actually 10)
    static const double H = 1.00; // Measurement map scalar
    static double Q = 10;         //initial estimated covariance
    static double P = 0;          //initial error covariance (must be 0)
    static double U_hat = 0;      // initial estimated state (assume we dont know)
    static double K = 0;          // Kalman Gain

    //begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    //Update error covariance
    P = (1 - K * H) * P + Q;

    // Return the estimate of U_hat
    return U_hat;
}

float KALMAN_GyroY(float U)
{
    // U = Noisy Measurement, returns U_hat = Filter Estimate

    //constants (static)
    static const double R = 40;   // Noise covariance (its actually 10)
    static const double H = 1.00; // Measurement map scalar
    static double Q = 10;         //initial estimated covariance
    static double P = 0;          //initial error covariance (must be 0)
    static double U_hat = 0;      // initial estimated state (assume we dont know)
    static double K = 0;          // Kalman Gain

    //begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    //Update error covariance
    P = (1 - K * H) * P + Q;

    // Return the estimate of U_hat
    return U_hat;
}

float KALMAN_GyroZ(float U)
{
    // U = Noisy Measurement, returns U_hat = Filter Estimate

    //constants (static)
    static const double R = 40;   // Noise covariance (its actually 10)
    static const double H = 1.00; // Measurement map scalar
    static double Q = 10;         //initial estimated covariance
    static double P = 0;          //initial error covariance (must be 0)
    static double U_hat = 0;      // initial estimated state (assume we dont know)
    static double K = 0;          // Kalman Gain

    //begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    //Update error covariance
    P = (1 - K * H) * P + Q;

    // Return the estimate of U_hat
    return U_hat;
}

void i2c_read_registers()
{
    // request a total of 14 registers
    Wire.requestFrom(MPU_addr, 14);
    AccX = (float)(Wire.read() << 8 | Wire.read()) / 16384; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AccY = (float)(Wire.read() << 8 | Wire.read()) / 16384; // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AccZ = (float)(Wire.read() << 8 | Wire.read()) / 16384; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    // accelReadCounter++;

    // Correcting with error readings
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;

    AccX = KALMAN_AccX(AccX);
    AccY = KALMAN_AccY(AccY);
    AccZ = KALMAN_AccZ(AccZ);

    // AccX = Kalman_Ax;
    Tmp = (float)((Wire.read() << 8 | Wire.read()) / 340) + 36.53; // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    // tempReadCounter++;

    // Calculating Roll and Pitch from the accelerometer data
    // accAngleX = (float)(atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58)
    // accAngleY = (float)(atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)

    GyroX = (float)(Wire.read() << 8 | Wire.read()) / 131.0; // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyroY = (float)(Wire.read() << 8 | Wire.read()) / 131.0; // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyroZ = (float)(Wire.read() << 8 | Wire.read()) / 131.0; // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    // gyroReadCounter++;

    // Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
    GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
    GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)

    GyroX = KALMAN_GyroX(GyroX);
    GyroY = KALMAN_GyroY(GyroY);
    GyroZ = KALMAN_GyroZ(GyroZ);

    // // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
    // gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
    // gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    // yaw =  yaw + GyroZ * elapsedTime;

    // // Complementary filter - combine accelerometer and gyro angle values
    // roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    // pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

void i2c_get_data()
{
    if ((lastPrint + PRINT_RATE) < millis())
    {
        i2c_read_registers();
        lastPrint = millis();
    }
}

void print_values(float x_accel, float y_accel, float z_accel, float temp, float x_gyro, float y_gyro, float z_gyro)
{

    char serial_output[100];

    sprintf(serial_output, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", x_accel, y_accel, z_accel, temp, x_gyro, y_gyro, z_gyro);
    Serial.println(serial_output);

    // Serial.print(roll);
    // Serial.print("/");
    // Serial.print(pitch);
    // Serial.print("/");
    // Serial.println(yaw);
}

void mpu_read()
{
    i2c_initialize();
    i2c_get_data();
}

void start_led_state_indicator()
{
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
}

void setup()
{
    system_update_cpu_freq(160);
    Serial.begin(Serial_Monitor_Baud); //start Serial console ouput
    //  GPSserial.begin(GPSBaud);//start softserial for GPS at defined baud rate

    start_led_state_indicator();
    while (!Serial)
    {
        ;
    }
    i2c_setup();
    delay(500);
    Serial.println("[PASSED] : IMU SETUP");
    calculate_IMU_error();
    Serial.println("[PASSED] : IMU ERROR CALCULATION");
    Serial.println("[READY] : SYSTEM READY");

    WiFi.begin(ssid, password);

    Serial.print("[READY] : Connecting to ");
    Serial.println(ssid);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("My Address: ");
    Serial.println(WiFi.localIP());

    // Start Websocket server
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
}

void loop()
{
    mpu_read();
    // print_values(AccX, AccY, AccZ, Tmp, GyroX, GyroY, GyroZ);
    webSocket.loop();

    // while (GPSserial.available()>0)
    // {
    //   Serial.write(GPSserial.read());
    // }
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    // delay(500);
    // digitalWrite(LED_BLUE, LOW);
    // digitalWrite(LED_RED, LOW);
    // digitalWrite(LED_GREEN, LOW);
    // delay(500);
}
