#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <WebSocketsServer.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <ServoInput.h>
#include "kalman_filter.h"

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

#define IMU_SS 5
#define BARO_SS 4

// Actuator COnfiguration
const int servo1Pin = 13;
const int servo2Pin = 12;
#define USE_PWM_RX
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

// Actuators
Servo L_WING_ACT;
Servo R_WING_ACT;

struct RX_Channel
{
  int PWM_MAX;
  int PWM_MIN;
  int PWM_CENTRE;
  int PWM_RAW;
  int PWM_F;
  float ANGLE;
  float ANGLE_MAX;
  float ANGLE_MIN;
  bool Connected;
};

// Channel1 Setup
// Receiver Configuration
const int ch1Pin = RX_CHANNEL_1; // channel 1
const int CH1_PulseMin = 950; // microseconds (us)
const int CH1_PulseMax = 2050; // Ideal values for your servo can be found with the "Calibration" example
float channel_1_fs = 1500; // Midpoint
// Channel2 Setup
const int ch2Pin = RX_CHANNEL_2; // channel 2
const int CH2_PulseMin = 950;    // microseconds (us)
const int CH2_PulseMax = 2050; // Ideal values for your servo can be found with the "Calibration" example
float channel_2_fs = 1500;  // Midpoint
// Channel3 Setup
const int ch3Pin = RX_CHANNEL_3; // channel 3
const int CH3_PulseMin = 950;    // microseconds (us)
const int CH3_PulseMax = 2050;   // Ideal values for your servo can be found with the "Calibration" example
float channel_3_fs = 1000;       // Manual

ServoInputPin<ch1Pin> RX_CH1_PIN(CH1_PulseMin, CH1_PulseMax);
ServoInputPin<ch2Pin> RX_CH2_PIN(CH2_PulseMin, CH2_PulseMax);
ServoInputPin<ch3Pin> RX_CH3_PIN(CH3_PulseMin, CH3_PulseMax);

RX_Channel RX_CH1;
RX_Channel RX_CH2;
RX_Channel RX_CH3;

#define GPSBaud 9600 //GPS Baud rate
#define Serial_Monitor_Baud 115200

#define RAD2DEG 57.29578
#define DEG2RAD 0.01745
#define M_PI 3.14159265358979323846

// For stats that happen every 5 seconds
unsigned long last = 0UL;

const char *ssid = "baca";
const char *password = "randy053";

// Sensors
TinyGPSPlus gps;
MPU9250 IMU(SPI, IMU_SS);
Adafruit_BMP280 bmp(BARO_SS);
WebSocketsServer webSocket = WebSocketsServer(80);

int status;
static uint32_t t_delta = 0;
uint32_t t_start = 0;
float sample_rate;

float acc[3], gyro[3], mag[3];
float lin_a[3];
float qrt[4] = {1.00f, 0.00f, 0.00f, 0.00f};

float a_offset[3] = {0.99, -0.81, -1.03};
float g_offset[3] = {-0.064, 0.20, -0.00};

double acc_k[3], gyro_k[3], mag_k[3];
float pitch_k, roll_k, heading_k;

//Variables for AHRS and quaternion
float heading;
bool USE_TRUE_NORTH = false;

unsigned long Now = 0;
float deltat = 0.0f, sum = 0.0f; // integration interval for both filter schemes
unsigned long lastUpdate = 0;
unsigned long count = 0, sumCount = 0;
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold intergal error in mahony filter

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f);  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f                          // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float pitchError;
float rollError;

float pitch, yaw, roll; // MAIN ATTITUDE VAIRABLES
float pitch_a, roll_a;
float pitch_q, yaw_q, roll_q;

float magnetic_declination = -12.97; // Ottawa, Nov 23 2020
float temp_imu, temp, pressure, altitude;

char buffer_udp_out[150];
char buffer_udp_in[150];
char buffer_serial_out[200];

// Called when receiving websocket packet
void onWebSocketEvent(u_int num, WStype_t type, uint8_t *payload, size_t length)
{

  // Determine type of Websocket event
  switch (type)
  {
  case (WStype_DISCONNECTED):
  {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Disconnected.\n", num);
    Serial.println(ip.toString());
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

    sprintf(buffer_udp_out, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], qrt[0], qrt[1], qrt[2], qrt[3], pressure, altitude, temp, pitch_k, roll_k, heading_k);
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

void gps_update()
{
  // Dispatch incoming characters
  while (Serial2.available() > 0)
    gps.encode(Serial2.read());

  if (gps.location.isUpdated())
  {
    Serial.print(F("LOCATION   Fix Age="));
    Serial.print(gps.location.age());
    Serial.print(F("ms Raw Lat="));
    Serial.print(gps.location.rawLat().negative ? "-" : "+");
    Serial.print(gps.location.rawLat().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLat().billionths);
    Serial.print(F(" billionths],  Raw Long="));
    Serial.print(gps.location.rawLng().negative ? "-" : "+");
    Serial.print(gps.location.rawLng().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLng().billionths);
    Serial.print(F(" billionths],  Lat="));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" Long="));
    Serial.println(gps.location.lng(), 6);
  }

  else if (gps.date.isUpdated())
  {
    Serial.print(F("DATE       Fix Age="));
    Serial.print(gps.date.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.date.value());
    Serial.print(F(" Year="));
    Serial.print(gps.date.year());
    Serial.print(F(" Month="));
    Serial.print(gps.date.month());
    Serial.print(F(" Day="));
    Serial.println(gps.date.day());
  }

  else if (gps.time.isUpdated())
  {
    Serial.print(F("TIME       Fix Age="));
    Serial.print(gps.time.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.time.value());
    Serial.print(F(" Hour="));
    Serial.print(gps.time.hour());
    Serial.print(F(" Minute="));
    Serial.print(gps.time.minute());
    Serial.print(F(" Second="));
    Serial.print(gps.time.second());
    Serial.print(F(" Hundredths="));
    Serial.println(gps.time.centisecond());
  }

  else if (gps.speed.isUpdated())
  {
    Serial.print(F("SPEED      Fix Age="));
    Serial.print(gps.speed.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.speed.value());
    Serial.print(F(" Knots="));
    Serial.print(gps.speed.knots());
    Serial.print(F(" MPH="));
    Serial.print(gps.speed.mph());
    Serial.print(F(" m/s="));
    Serial.print(gps.speed.mps());
    Serial.print(F(" km/h="));
    Serial.println(gps.speed.kmph());
  }

  else if (gps.course.isUpdated())
  {
    Serial.print(F("COURSE     Fix Age="));
    Serial.print(gps.course.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.course.value());
    Serial.print(F(" Deg="));
    Serial.println(gps.course.deg());
  }

  else if (gps.altitude.isUpdated())
  {
    Serial.print(F("ALTITUDE   Fix Age="));
    Serial.print(gps.altitude.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.altitude.value());
    Serial.print(F(" Meters="));
    Serial.print(gps.altitude.meters());
    Serial.print(F(" Miles="));
    Serial.print(gps.altitude.miles());
    Serial.print(F(" KM="));
    Serial.print(gps.altitude.kilometers());
    Serial.print(F(" Feet="));
    Serial.println(gps.altitude.feet());
  }

  else if (gps.satellites.isUpdated())
  {
    Serial.print(F("SATELLITES Fix Age="));
    Serial.print(gps.satellites.age());
    Serial.print(F("ms Value="));
    Serial.println(gps.satellites.value());
  }

  else if (gps.hdop.isUpdated())
  {
    Serial.print(F("HDOP       Fix Age="));
    Serial.print(gps.hdop.age());
    Serial.print(F("ms raw="));
    Serial.print(gps.hdop.value());
    Serial.print(F(" hdop="));
    Serial.println(gps.hdop.hdop());
  }

  else if (millis() - last > 5000)
  {
    Serial.println();
    if (gps.location.isValid())
    {
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
      double distanceToLondon =
          TinyGPSPlus::distanceBetween(
              gps.location.lat(),
              gps.location.lng(),
              LONDON_LAT,
              LONDON_LON);
      double courseToLondon =
          TinyGPSPlus::courseTo(
              gps.location.lat(),
              gps.location.lng(),
              LONDON_LAT,
              LONDON_LON);

      Serial.print(F("LONDON     Distance="));
      Serial.print(distanceToLondon / 1000, 6);
      Serial.print(F(" km Course-to="));
      Serial.print(courseToLondon, 6);
      Serial.print(F(" degrees ["));
      Serial.print(TinyGPSPlus::cardinal(courseToLondon));
      Serial.println(F("]"));
    }

    Serial.print(F("DIAGS      Chars="));
    Serial.print(gps.charsProcessed());
    Serial.print(F(" Sentences-with-Fix="));
    Serial.print(gps.sentencesWithFix());
    Serial.print(F(" Failed-checksum="));
    Serial.print(gps.failedChecksum());
    Serial.print(F(" Passed-checksum="));
    Serial.println(gps.passedChecksum());

    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));

    last = millis();
    Serial.println();
  }
}

void init_WiFi()
{
  WiFi.begin(ssid, password);

  Serial.print("[READY] : Connecting to ");
  Serial.println(ssid);

  int wifi_connection_attempts = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
    wifi_connection_attempts++;
    if (wifi_connection_attempts > 20)
    {
      Serial.println("[ERROR] : Wifi connection attempt limit exceeded (restarting) ");
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      delay(400);
      ESP.restart();
    }
  }
  Serial.println();
  Serial.print("[INFO] : Connected @ ");
  Serial.println(WiFi.localIP());

  // Start Websocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void start_websocket_loop()
{
  webSocket.loop();
}

float convert_rad_s_to_deg_s(float rad_s)
{
  return rad_s * 57.29577951308;
}

int print2(float x, float y)
{
  Serial.print(x);
  Serial.print("\t\t");
  Serial.println(y);

  return 0;
}

int print3(float x, float y, float z)
{
  Serial.print(x);
  Serial.print("\t\t");
  Serial.print(y);
  Serial.print("\t\t");
  Serial.println(z);

  return 1;
}

void setFullScaleRanges()
{
  // status = IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  status = IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  delay(10);
  // status = IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  // status = IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  status = IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
}

void setBiasesAndScaleFactors()
{
  IMU.setGyroBiasX_rads(0.0);
  delay(10);
  IMU.setGyroBiasY_rads(0.0);
  delay(10);
  IMU.setGyroBiasZ_rads(0.0);
  delay(10);
  IMU.setMagCalX(25.61, 1.07);
  delay(10);
  IMU.setMagCalY(20.66, 0.96);
  delay(10);
  IMU.setMagCalZ(0.928, 0.97);
}

void sensors_init()
{
  // start communication with IMU
  status = IMU.begin();
  if (!status)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
      Serial.println("[ERROR] : Failed to connect to IMU");
    }
  }
  else
  {
    setFullScaleRanges();
    setBiasesAndScaleFactors();
    Serial.println("[SETUP] : IMU Scale Factor and Bias SET ");
  }
  // start communication to Barometer
  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1)
    {
      Serial.println("[ERROR] : Failed to connect to Barometer");
      ;
    }
  }
  else
  {
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_NONE, /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_NONE, /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_OFF,    /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    Serial.println("[SETUP] : Barometer Sampling Settings SET");
  }
}

void imu_calibrate()
{
  Serial.print("[INFO]: Accel calibraton started... Returned: ");
  Serial.println(IMU.calibrateAccel());
  Serial.print("Accel Bias XYZ: ");
  print3(IMU.getAccelBiasX_mss(), IMU.getAccelBiasY_mss(), IMU.getAccelBiasZ_mss());
  Serial.print("Accel Scale XYZ: ");
  print3(IMU.getAccelScaleFactorX(), IMU.getAccelScaleFactorY(), IMU.getAccelScaleFactorZ());

  delay(1000);

  Serial.print("[INFO]: Gyro calibration started... Returned: ");
  Serial.println(IMU.calibrateGyro());
  Serial.print("Gyro Bias XYZ: ");
  print3(IMU.getGyroBiasX_rads(), IMU.getGyroBiasY_rads(), IMU.getGyroBiasZ_rads());

  delay(1000);

  Serial.print("[INFO]: Mag calibration started... Returned: ");
  Serial.println(IMU.calibrateMag());
  Serial.print("Mag Bias XYZ: ");
  print3(IMU.getMagBiasX_uT(), IMU.getMagBiasY_uT(), IMU.getMagBiasZ_uT());
  Serial.print("Mag Scale XYZ: ");
  print3(IMU.getMagScaleFactorX(), IMU.getMagScaleFactorY(), IMU.getMagScaleFactorZ());

  delay(1000);
}

// Get resting accel & gyro offset values
int update_imu_resting_offsets()
{
  float a_off[3], g_off[3];

  for (int i = 0; i < 150; i++)
  {
    IMU.readSensor();
    a_off[0] += IMU.getAccelX_mss();
    a_off[1] += IMU.getAccelY_mss();
    a_off[2] += IMU.getAccelZ_mss();

    g_off[0] += convert_rad_s_to_deg_s(IMU.getGyroX_rads());
    g_off[1] += convert_rad_s_to_deg_s(IMU.getGyroY_rads());
    g_off[2] += convert_rad_s_to_deg_s(IMU.getGyroZ_rads());

    delay(5);
  }
  for (int j = 0; j < 3; j++)
  {
    a_off[j] = (a_off[j] ? a_off[j] : 0.001) / 150;
    g_off[j] = (g_off[j] ? g_off[j] : 0.001) / 150;
  }
  Serial.println("\nAverage values below");
  print3(a_off[0], a_off[1], a_off[2]);
  Serial.println();

  return 1;
}

void imu_update()
{

  static uint32_t prev_ms = millis();

  if (IMU.readSensor())
  {
    if ((millis() - prev_ms) > 1){

      acc[0] = IMU.getAccelX_mss() + a_offset[0];
      acc[1] = IMU.getAccelY_mss() + a_offset[1];
      acc[2] = IMU.getAccelZ_mss() + a_offset[2];

      gyro[0] = IMU.getGyroX_rads();
      gyro[0] = convert_rad_s_to_deg_s(gyro[0]) + g_offset[0];
      gyro[1] = IMU.getGyroY_rads();
      gyro[1] = convert_rad_s_to_deg_s(gyro[1]) + g_offset[1];
      gyro[2] = IMU.getGyroZ_rads();
      gyro[2] = convert_rad_s_to_deg_s(gyro[2]) + g_offset[2];

      mag[0] = IMU.getMagX_uT();
      mag[1] = IMU.getMagY_uT();
      mag[2] = IMU.getMagZ_uT();

      temp_imu = IMU.getTemperature_C();
      
      prev_ms = millis();
    }
    
  }
}

void baro_update()
{
  static uint32_t prev_ms = millis();

  if ((millis() - prev_ms) > 0)
  {
    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(1026);
    prev_ms = millis();
  }
}

// Updates all the Kalman filter values
void update_kalman_filter()
{
  acc_k[0] = kalman_ax((double)acc[0]);
  acc_k[1] = kalman_ay((double)acc[1]);
  acc_k[2] = kalman_az((double)acc[2]);

  gyro_k[0] = kalman_gx((double)gyro[0]);
  gyro_k[1] = kalman_gy((double)gyro[1]);
  gyro_k[2] = kalman_gz((double)gyro[2]);

  mag_k[0] = kalman_mx((double)mag[0]);
  mag_k[1] = kalman_my((double)mag[1]);
  mag_k[2] = kalman_mz((double)mag[2]);

  heading_k = kalman_compass((double)yaw);
  pitch_k = kalman_pitch((double)pitch);
  roll_k = kalman_roll((double)roll);
}

void update_body_velocity()
{
  ;
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = qrt[0], q2 = qrt[1], q3 = qrt[2], q4 = qrt[3]; // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f)
    return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f)
    return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;

  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
  norm = 1.0f / norm;

  qrt[0] = q1 * norm;
  qrt[1] = q2 * norm;
  qrt[2] = q3 * norm;
  qrt[3] = q4 * norm;
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = qrt[0], q2 = qrt[1], q3 = qrt[2], q4 = qrt[3];
  //short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // ----- Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // ----- Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f)
    return;           // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // ----- Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f)
    return;           // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // ----- Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // ----- Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  //  ----- Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex; // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f; // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // ----- Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // ----- Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // ----- Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  qrt[0] = q1 * norm;
  qrt[1] = q2 * norm;
  qrt[2] = q3 * norm;
  qrt[3] = q4 * norm;
}

void update_quaternion()
{
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  // MAHONY QUARTERNION FILTER
  // MahonyQuaternionUpdate(acc_k[0], acc_k[1], acc_k[2], gyro_k[0] * DEG_TO_RAD, gyro_k[1] * DEG_TO_RAD, gyro_k[2] * DEG_TO_RAD, mag[0], mag[1], mag[2]);

  // MADGWICK QUARTERNION FILTER
  MadgwickQuaternionUpdate(acc_k[0], acc_k[1], acc_k[2], gyro_k[0] * DEG_TO_RAD, gyro_k[1] * DEG_TO_RAD, gyro_k[2] * DEG_TO_RAD, mag_k[0], mag_k[1], mag_k[2]);

  // Calculate the yaw
  yaw_q = atan2(2.0f * (qrt[1] * qrt[2] + qrt[0] * qrt[3]), qrt[0] * qrt[0] + qrt[1] * qrt[1] - qrt[2] * qrt[2] - qrt[3] * qrt[3]);
  heading = yaw_q * RAD2DEG;
  heading -= magnetic_declination; 

  // Calculate pitch and roll
  pitch_q = asin(2.0f * (qrt[1] * qrt[3] - qrt[0] * qrt[2]));
  roll_q = -atan2(2.0f * (qrt[0] * qrt[1] + qrt[2] * qrt[3]), qrt[0] * qrt[0] - qrt[1] * qrt[1] - qrt[2] * qrt[2] + qrt[3] * qrt[3]);

  // convert to degrees
  pitch_q *= RAD_TO_DEG;
  roll_q *= RAD_TO_DEG;
  yaw_q *= RAD_TO_DEG;
  yaw_q -= magnetic_declination;

  // Normalizing Roll to +(0-180) upright and -(0-180) upside down.
  if (roll_q > 0)
  {
    roll_q = roll_q - 180;
  }
  else
  {
    roll_q = 180 + roll_q;
  }

  if (heading < 0)
    heading += 360.0; // Yaw goes negative between 180 amd 360 degrees
  if (USE_TRUE_NORTH == true)
    heading += magnetic_declination; // Calculate True North
  if (heading < 0)
    heading += 360.0; // Allow for under|overflow
  if (heading >= 360)
    heading -= 360.0;

  // Normalizing Heading to compass convention N.W.S.E clockwise
  heading = 360 - heading;
}

// Update Yaw, Pitch and Roll
void update_YPR()
{
  pitch_a = -atan2(acc[0] / 9.8, acc[2] / 9.8) / 2 / 3.141592654 * 360;
  roll_a = -atan2(acc[1] / 9.8, acc[2] / 9.8) / 2 / 3.141592654 * 360;

  if (pitch_a < 0)
  {
    pitch_a = 180 + pitch_a;
  }
  else
  {
    pitch_a = -(180 - pitch_a);
  }
  if (roll_a < 0)
  {
    roll_a = 180 + roll_a;
  }
  else
  {
    roll_a = -(180 - roll_a);
  }

  update_quaternion();
  
  pitch = pitch_q; // Pitch from Quaternion
  roll = roll_q;   // Roll from Quaternion
  yaw = heading;   // Yaw from Quaternion

  pitchError = pitch_q * 0.35 + pitch_a * 0.65;
  rollError = roll_q * 0.35 + roll_a * 0.65;

}

void print_serial_buffer(char *serial_buff)
{
  sprintf(serial_buff, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], qrt[0], qrt[1], qrt[2], qrt[3], pressure, altitude, temp, pitch, roll, yaw, t_delta, sample_rate, acc_k[0], acc_k[1], acc_k[2], gyro_k[0], gyro_k[1], gyro_k[2], mag_k[0], mag_k[1], mag_k[2], heading_k, pitch_k, roll_k);
  // Serial.println(serial_buff);

  // sprintf(serial_buff, "PRY: %.3f,%.3f,%.3f PRY_K: %.3f,%.3f,%.3f BMP: %.3f,%.3f,%.3f", pitch, roll, yaw, pitch_k, roll_k, heading_k, pressure, altitude, temp);
  Serial.println(serial_buff);
}

void BUZZER_1(){
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void BUZZER_2()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void update_RX_Channels (){
  int CH_1_Raw = RX_CH1_PIN.getPulseRaw();
  int CH_2_Raw = RX_CH2_PIN.getPulseRaw();
  int CH_3_Raw = RX_CH3_PIN.getPulseRaw();

  if (CH1_PulseMin < CH_1_Raw && CH1_PulseMax > CH_1_Raw)
  {
    RX_CH1.PWM_RAW = CH_1_Raw;
  }
  if (CH2_PulseMin < CH_2_Raw && CH2_PulseMax > CH_2_Raw)
  {
    RX_CH2.PWM_RAW = CH_2_Raw;
  }
  if (CH3_PulseMin < CH_3_Raw && CH3_PulseMax > CH_3_Raw)
  {
    RX_CH3.PWM_RAW = CH_3_Raw;
  }
}

void update_RX_Indicators(){
  if (950 < RX_CH3.PWM_RAW && RX_CH3.PWM_RAW < 1050)
  {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);

  }
  if (1450 < RX_CH3.PWM_RAW && RX_CH3.PWM_RAW < 1550)
  {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
  }
  if (1950 < RX_CH3.PWM_RAW && RX_CH3.PWM_RAW < 2050)
  {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
  }
} 

void setup()
{
  // Setup Indicators
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // BUILTIN LED ON during Setup
  digitalWrite(LED_BUILTIN, HIGH);
  BUZZER_1();

  Serial.begin(Serial_Monitor_Baud);
  Serial2.begin(GPSBaud, SERIAL_8N1, RXD2, TXD2);

  while (!Serial)
  {
  }

  sensors_init();
  init_WiFi();

  /** 
    * Calibration? [Last Calibrated Nov 28. 2020]
    * imu_calibrate();
    * update_imu_resting_offsets();
  **/

  L_WING_ACT.attach(L_W_SERVO, Servo::CHANNEL_NOT_ATTACHED, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, MIN_SERVO_PWM, MAX_SERVO_PWM);
  R_WING_ACT.attach(R_W_SERVO, Servo::CHANNEL_NOT_ATTACHED, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, MIN_SERVO_PWM, MAX_SERVO_PWM);

  //Turn Setup Indicator OFF
  digitalWrite(LED_BUILTIN, LOW);
  BUZZER_2();
}

void loop()
{
  // Start timer
  t_start = micros();

  // Starts ws communication loop
  // start_websocket_loop();

  // read the IMU values
  imu_update();
  update_YPR();
  update_kalman_filter();
  // update_body_velocity();

  // Read the barometer
  // baro_update();

  // Update GPS reading if available
  // gps_update();

  update_RX_Channels();
  update_RX_Indicators();

  // L_WING_ACT.write(ch_1_servo_angle);
  // R_WING_ACT.write(ch_2_servo_angle);

  t_delta = micros() - t_start;
  if (t_delta > 0)
  {
    sample_rate = (float)t_delta;
    sample_rate = 1 / (sample_rate / 1000000);
  }

  // print_serial_buffer(buffer_serial_out);

  Serial.print(pitch_a);
  Serial.print(",");
  Serial.print(roll_a);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitchError);
  Serial.print(",");
  Serial.println(rollError);

  // Serial.print(RX_CH1_PIN.getPulseRaw());
  // Serial.print(",");
  // Serial.print(RX_CH2_PIN.getPulseRaw());
  // Serial.print(",");
  // Serial.print(RX_CH3_PIN.getPulseRaw());
  // Serial.print(",");
  // Serial.println(sample_rate);

}