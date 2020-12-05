#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include "kalman_filter.h"

#define RAD2DEG 57.29578
#define DEG2RAD 0.01745
#define M_PI 3.14159265358979323846

MPU9250 IMU(SPI, D0);
Adafruit_BMP280 bmp(D1);

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
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
unsigned long lastUpdate = 0;
unsigned long count = 0, sumCount = 0;
float eInt[3] = {0.0f, 0.0f, 0.0f};     // vector to hold intergal error in mahony filter

// Mahony free parameters
//#define Kp 2.0f * 5.0f                // original Kp proportional feedback parameter in Mahony filter and fusion scheme
#define Kp 40.0f                        // Kp proportional feedback parameter in Mahony filter and fusion scheme
#define Ki 0.0f                         // Ki integral parameter in Mahony filter and fusion scheme

float pitch, yaw, roll;                 // MAIN ATTITUDE VAIRABLES
float pitch_a, roll_a;
float pitch_q, yaw_q, roll_q;

float magnetic_declination = -12.97;    // Ottawa, Nov 23 2020
float temp_imu, temp, pressure, altitude;

char buffer_serial_out[200];

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
  status = IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  delay(10);
  status = IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
}

void setBiasesAndScaleFactors()
{
  IMU.setGyroBiasX_rads(-0.01);
  delay(10);
  IMU.setGyroBiasY_rads(0.02);
  delay(10);
  IMU.setGyroBiasZ_rads(0);
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
      ;
    }
  }
  else
  {
    setFullScaleRanges();
    setBiasesAndScaleFactors();
  }
  // start communication to Barometer
  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1)
    {
      ;
    }
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_NONE, /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_NONE, /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,    /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
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

  if (IMU.readSensor())
  {
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
  }
}

void baro_update()
{
  static uint32_t prev_ms = millis();

  if ((millis() - prev_ms) > 5)
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

  /*
    Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    This is ok by aircraft orientation standards!
    Pass gyro rate as rad/s
  */
  MahonyQuaternionUpdate(acc[0], acc[1], acc[2], gyro[0] * DEG_TO_RAD, gyro[1] * DEG_TO_RAD, gyro[2] * DEG_TO_RAD, mag[0], mag[1], mag[2]);

  // Calculate the yaw
  yaw_q = atan2(2.0f * (qrt[1] * qrt[2] + qrt[0] * qrt[3]), qrt[0] * qrt[0] + qrt[1] * qrt[1] - qrt[2] * qrt[2] - qrt[3] * qrt[3]);
  heading = yaw_q * RAD2DEG;

  // Calculate pitch and roll
  pitch_q = asin(2.0f * (qrt[1] * qrt[3] - qrt[0] * qrt[2]));
  roll_q = -atan2(2.0f * (qrt[0] * qrt[1] + qrt[2] * qrt[3]), qrt[0] * qrt[0] - qrt[1] * qrt[1] - qrt[2] * qrt[2] + qrt[3] * qrt[3]);

  // convert to degrees
  pitch_q *= RAD_TO_DEG;
  roll_q *= RAD_TO_DEG;
  yaw_q *= RAD_TO_DEG;

  // Normalizing Roll to +(0-180) upright and -(0-180) upside down.
  if (roll_q > 0){
    roll_q = roll_q - 180;
  } else {
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

  pitch = pitch_q;        // Pitch from Quaternion
  roll = roll_q;          // Roll from Quaternion
  yaw = heading;          // Yaw from Quaternion
}

void setup()
{
  // serial to display data
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }
  sensors_init();

  /** 
    * Calibration? [Last Calibrated Nov 28. 2020]
    * imu_calibrate();
    * update_imu_resting_offsets();
  **/
}

void loop()
{
  // Start timer
  t_start = millis();

  // read the IMU values
  imu_update();
  update_YPR();
  update_kalman_filter();
  update_body_velocity();

  // Read the barometer 
  baro_update();

  t_delta = millis() - t_start;
  if (t_delta > 0)
  {
    sample_rate = (float)t_delta;
    sample_rate = 1 / (sample_rate / 1000);
  }

  sprintf(buffer_serial_out, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], qrt[0], qrt[1], -qrt[2], qrt[3], pressure, altitude, temp, pitch, roll, yaw, t_delta, sample_rate, acc_k[0], acc_k[1], acc_k[2], gyro_k[0], gyro_k[1], gyro_k[2], mag_k[0], mag_k[1], mag_k[2], heading_k, pitch_k, roll_k);
  Serial.println(buffer_serial_out);
}
