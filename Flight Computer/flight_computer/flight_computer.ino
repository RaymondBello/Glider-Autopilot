#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>

#define LED_RED D5
#define LED_GREEN D6
#define LED_BLUE D7

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define GPSBaud 9600 //GPS Baud rate
#define Serial_Monitor_Baud 115200

float a_offset[3], g_offset[3];
float acc[3], gyro[3], mag[3];
float qrt[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float pitch, yaw, roll;
float temp, pressure, altitude;

MPU9250 mpu;
Adafruit_BMP280 bmp; // I2C
WebSocketsServer webSocket = WebSocketsServer(80);

// State Constants
const int CURRENT_STATE = 0;
int STATE_POOL[7] = {0, 1, 2, 3, 4, 5, 6};

const char *ssid = "baca";
const char *password = "randy053";

char buffer_udp_out[150];
char buffer_udp_in[150];
char buffer_serial_out[150];

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

        sprintf(buffer_udp_out, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], qrt[0], qrt[1], qrt[2], qrt[3], pressure, altitude, temp, pitch, roll, yaw);
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

void start_led_state_indicator()
{
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
}

void I2C_sensors_begin()
{
  Wire.begin();
  mpu.setup();
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void init_WiFi()
{
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
  Serial.print(WiFi.localIP());
  Serial.println();

  // Start Websocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void serial_print_GPS_values()
{
  // while (GPSserial.available()>0)
  // {
  //   Serial.write(GPSserial.read());
  // }
}

void update_led_state_indicator()
{
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  // delay(500);
  // digitalWrite(LED_BLUE, LOW);
  // digitalWrite(LED_RED, LOW);
  // digitalWrite(LED_GREEN, LOW);
  // delay(500);
}

void start_websocket_loop()
{
  webSocket.loop();
}

void IMU_update()
{
//  memset(buffer_serial_out, 0, sizeof buffer_serial_out);
  static uint32_t prev_ms = millis();

  if ((millis() - prev_ms) > 16)
  {
    mpu.update();

    acc[0] = mpu.getAcc(0) - a_offset[0];
    acc[1] = mpu.getAcc(1) - a_offset[1];
    acc[2] = mpu.getAcc(2) - a_offset[2];

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

//  sprintf(buffer_serial_out, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], qrt[0], qrt[1], qrt[2], qrt[3], pressure, altitude, temp, pitch, roll, yaw);
//  Serial.println(buffer_serial_out);
}

void baro_update()
{
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(1026);
}

void IMU_calibrate(){
   float a_raw[3],g_raw[3];
   int i;

   for (i=0; i<100; i++){
    mpu.update();

    a_raw[0] += mpu.getAcc(0);
    a_raw[1] += mpu.getAcc(1);
    a_raw[2] += mpu.getAcc(2);

    g_raw[0] += mpu.getGyro(0);
    g_raw[1] += mpu.getGyro(1);
    g_raw[2] += mpu.getGyro(2);
    delay(2);
   }
   
   a_offset[0] = a_raw[0]/100;
   a_offset[1] = a_raw[1]/100;
   a_offset[2] = a_raw[2]/100;

   g_offset[0] = g_raw[0]/100;
   g_offset[1] = g_raw[1]/100;
   g_offset[2] = g_raw[2]/100;  

   Serial.print("Offsets: ");
   Serial.print(a_offset[0]); Serial.print(",");
   Serial.print(a_offset[1]); Serial.print(",");
   Serial.print(a_offset[2]); Serial.print(",");
   Serial.print(g_offset[0]); Serial.print(",");
   Serial.print(g_offset[1]); Serial.print(",");
   Serial.print(g_offset[2]); Serial.println(" ");
}

void setup()
{
  system_update_cpu_freq(160);
  Serial.begin(Serial_Monitor_Baud);
  start_led_state_indicator();
  while (!Serial) {;}
  I2C_sensors_begin();
  init_WiFi();
  IMU_calibrate();
}

void loop()
{
  start_websocket_loop();
  update_led_state_indicator();
  IMU_update();
  baro_update();
  delay(10);
}
