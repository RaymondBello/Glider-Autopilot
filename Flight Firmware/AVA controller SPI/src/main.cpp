#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include "kalman_filter.h"

MPU9250 IMU(SPI, D0);
Adafruit_BMP280 bmp(D1); 

int status;
static uint32_t t_delta = 0;
uint32_t t_start = 0;
float sample_rate;

float a_offset[3], g_offset[3];
float acc[3], gyro[3], mag[3];
float lin_a[3];
float qrt[4] = {1.0f, 0.0f, 0.0f, 0.0f};
double acc_k[3], gyro_k[3], mag_k[3];
float pitch, yaw, roll;
float temp_imu, temp, pressure, altitude;

char buffer_serial_out[200];

// #define BMP_CS (5)
// #define BMP_SCK (14)
// #define BMP_MISO (12)
// #define BMP_MOSI (13)
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void sensors_init(){

  // start communication with IMU
  status = IMU.begin();
  if (status < 0)
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

void imu_calibrate(){
  ;
}

void imu_update(){

  IMU.readSensor();

  acc[0] = IMU.getAccelX_mss();
  acc[1] = IMU.getAccelY_mss();
  acc[2] = IMU.getAccelZ_mss();

  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();

  mag[0] = IMU.getMagX_uT();
  mag[1] = IMU.getMagY_uT();
  mag[2] = IMU.getMagZ_uT();

  temp_imu = IMU.getTemperature_C();
}

void baro_update(){
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
}

void setup()
{
  // serial to display data
  Serial.begin(115200);
  while (!Serial)
  {
  }
  sensors_init();
  // Calibration?
}

void loop()
{
  // Start timer
  t_start = millis();

  // read the sensor
  imu_update();
  update_kalman_filter();
  baro_update();

  t_delta = millis() - t_start;
  if (t_delta > 0)
  {
    sample_rate = (float)t_delta;
    sample_rate = 1 / (sample_rate / 1000);
  }

  sprintf(buffer_serial_out, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %.1f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], qrt[0], qrt[1], -qrt[2], qrt[3], pressure, altitude, temp, pitch, roll, yaw, t_delta, sample_rate, acc_k[0], acc_k[1], acc_k[2], gyro_k[0], gyro_k[1], gyro_k[2], mag_k[0], mag_k[1], mag_k[2]);
  Serial.println(buffer_serial_out);
  
  // Serial.printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", sample_rate, acc_k[0], acc_k[1], acc_k[2], gyro_k[0], gyro_k[1], gyro_k[2], mag_k[0], mag_k[1], mag_k[2]);
  // Serial.println();

  // display the data
}

