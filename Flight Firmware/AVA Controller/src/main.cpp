#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include "kalman_filter.h"

#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

#define Serial_Monitor_Baud 115200

float a_offset[3], g_offset[3];
float acc[3], gyro[3], mag[3];
float lin_a[3];
float qrt[4] = {1.0f, 0.0f, 0.0f, 0.0f};
double acc_k[3], gyro_k[3], mag_k[3];
float pitch, yaw, roll;
float temp, pressure, altitude;
float sample_rate;

MPU9250 mpu;
Adafruit_BMP280 bmp; // I2C

static uint32_t t_delta = 0;
uint32_t t_start = 0;

char buffer_serial_out[200];

void I2C_sensors_begin()
{
  Wire.begin();
  mpu.setup();
  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1)
      ;
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void IMU_update()
{
  //  memset(buffer_serial_out, 0, sizeof buffer_serial_out);
  static uint32_t prev_ms = millis();

  if ((millis() - prev_ms) > 0)
  {
    mpu.update();

    acc[0] = mpu.getAcc(0); //- a_offset[0];
    acc[1] = mpu.getAcc(1); //- a_offset[1];
    acc[2] = mpu.getAcc(2); //- a_offset[2];

    lin_a[0] = mpu.getLinAccX();
    lin_a[1] = mpu.getLinAccY();
    lin_a[2] = mpu.getLinAccZ();

    gyro[0] = mpu.getGyro(0) - g_offset[0];
    gyro[1] = mpu.getGyro(1) - g_offset[1];
    gyro[2] = mpu.getGyro(2) - g_offset[2];

    mag[0] = mpu.getMag(0);
    mag[1] = mpu.getMag(1);
    mag[2] = mpu.getMag(2);

    qrt[0] = mpu.getQuaternion(0);
    qrt[1] = mpu.getQuaternion(1);
    qrt[2] = mpu.getQuaternion(2);
    qrt[3] = mpu.getQuaternion(3);

    roll = mpu.getRoll();
    pitch = mpu.getPitch();
    yaw = mpu.getYaw();

    prev_ms = millis();
  }
}

void IMU_calibrate()
{
  /**
   * IMU LAST CALIBRATED {11/20/2020 11:33PM}
   */
  delay(50);
  mpu.calibrateAccelGyro();
  // delay(50);
  // mpu.calibrateMag();

  float g_raw[3] = {};
  //, a_raw[3] ;
  int i;

  for (i = 0; i < 100; i++)
  {
    mpu.update();

    g_raw[0] += mpu.getGyro(0);
    g_raw[1] += mpu.getGyro(1);
    g_raw[2] += mpu.getGyro(2);

    delay(4);

    // a_raw[0] += mpu.getAcc(0);
    //   a_raw[1] += mpu.getAcc(1);
    //   a_raw[2] += mpu.getAcc(2);
  }
  // a_offset[0] = a_raw[0] / 100;
  // a_offset[1] = a_raw[1] / 100;
  // a_offset[2] = a_raw[2] / 100;

  g_offset[0] = g_raw[0] / 100;
  g_offset[1] = g_raw[1] / 100;
  g_offset[2] = g_raw[2] / 100;

  // Serial.print("Offsets: ");
  // Serial.print(a_offset[0]);
  // Serial.print(",");
  // Serial.print(a_offset[1]);
  // Serial.print(",");
  // Serial.print(a_offset[2]);
  // Serial.print(",");
  // Serial.print(g_offset[0]);
  // Serial.print(",");
  // Serial.print(g_offset[1]);
  // Serial.print(",");
  // Serial.print(g_offset[2]);
  // Serial.println(" ");
}

void baro_update()
{
  static uint32_t prev_ms = millis();

  if ((millis() - prev_ms) > 1)
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
  acc_k[0] = kalman_ax((double)lin_a[0]);
  acc_k[1] = kalman_ay((double)lin_a[1]);
  acc_k[2] = kalman_az((double)lin_a[2]);

  gyro_k[0] = kalman_gx((double)gyro[0]);
  gyro_k[1] = kalman_gy((double)gyro[1]);
  gyro_k[2] = kalman_gz((double)gyro[2]);

  mag_k[0] = kalman_mx((double)mag[0]);
  mag_k[1] = kalman_my((double)mag[1]);
  mag_k[2] = kalman_mz((double)mag[2]);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(Serial_Monitor_Baud);
  while (!Serial)
  {
    ;
  }
  I2C_sensors_begin();
  IMU_calibrate();
  Wire.setClock(400000L);
}

void loop()
{
  // Start timer
  t_start = millis();

  /*******************************
   *  Sensor update functions
   *******************************/
  baro_update();          // Barometer Update
  IMU_update();           // Update IMU values
  update_kalman_filter(); // Kalman filter updates

  t_delta = millis() - t_start;
  if (t_delta > 1)
  {
    sample_rate = (float)t_delta;
    sample_rate = 1 / (sample_rate / 1000);
  }

  /*
  sprintf(buffer_serial_out, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.1f, %.3f, %.3f, %.3f", lin_a[0], lin_a[1], lin_a[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], qrt[0], qrt[1], -qrt[2], qrt[3], pressure, altitude, temp, pitch, roll, yaw, t_delta, sample_rate, acc_k[0], acc_k[1], acc_k[2]);
  Serial.println(buffer_serial_out);
  */

  Serial.printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", sample_rate, acc_k[0], acc_k[1], acc_k[2], gyro_k[0], gyro_k[1], gyro_k[2], mag_k[0], mag_k[1], mag_k[2]);
  Serial.println();
  // delay(500);
}