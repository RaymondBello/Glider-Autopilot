#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebSocketsServer.h>

float a_offset[3], g_offset[3];
float acc[3], gyro[3], mag[3];
float lin_a[3];
float qrt[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float pitch, yaw, roll;
float temp, pressure, altitude;
float sample_rate;

WebSocketsServer webSocket = WebSocketsServer(80);

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }
  init_WiFi();
}

void loop() {
  // put your main code here, to run repeatedly:
  start_websocket_loop();
}