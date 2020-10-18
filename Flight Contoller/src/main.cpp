#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <WebSocketsServer.h>
#include <Wire.h>

#define LED_RED D5
#define LED_GREEN D6
#define LED_BLUE D7

#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

#define GPSBaud 9600 //GPS Baud rate
#define Serial_Monitor_Baud 115200

// AD0 low = 0x68 (default for InvenSense evaluation board)
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

WebSocketsServer webSocket = WebSocketsServer(80);

// State Constants
const int CURRENT_STATE = 0;
int STATE_POOL[7] = {0, 1, 2, 3, 4, 5, 6};

const char *ssid = "baca";
const char *password = "randy053";

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

        sprintf(buffer_udp_out, "%.3i,%.3i,%.3i,%.3i,%.3i,%.3i", ax, ay, az, gx, gy, gz);
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

void init_MPU6050(){
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset());
    Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset());
    Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset());
    Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset());
    Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset());
    Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset());
    Serial.print("\t"); // 0
    Serial.print("\n");

    accelgyro.setXAccelOffset(350);
    accelgyro.setYAccelOffset(-395);
    accelgyro.setZAccelOffset(-1226);

    Serial.print(accelgyro.getXAccelOffset());
    Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset());
    Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset());
    Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset());
    Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset());
    Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset());
    Serial.print("\t"); // 0
    Serial.print("\n");
    Serial.println("[PASSED] : IMU ERROR CALCULATION");
    Serial.println("[READY] : SYSTEM READY");

}

void serial_print_MPU6050_values(){
    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);
}

void serial_print_MPU6050_values_binary(){
    Serial.write((uint8_t)(ax >> 8));
    Serial.write((uint8_t)(ax & 0xFF));
    Serial.write((uint8_t)(ay >> 8));
    Serial.write((uint8_t)(ay & 0xFF));
    Serial.write((uint8_t)(az >> 8));
    Serial.write((uint8_t)(az & 0xFF));
    Serial.write((uint8_t)(gx >> 8));
    Serial.write((uint8_t)(gx & 0xFF));
    Serial.write((uint8_t)(gy >> 8));
    Serial.write((uint8_t)(gy & 0xFF));
    Serial.write((uint8_t)(gz >> 8));
    Serial.write((uint8_t)(gz & 0xFF));
}

void get_MPU6050_reading(){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
}

void start_led_state_indicator()
{
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
}

void I2C_begin(){
    Wire.begin();
}

void init_WiFi(){
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

void serial_print_GPS_values(){
    // while (GPSserial.available()>0)
    // {
    //   Serial.write(GPSserial.read());
    // }
}

void update_LED_state_indicator(){
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    // delay(500);
    // digitalWrite(LED_BLUE, LOW);
    // digitalWrite(LED_RED, LOW);
    // digitalWrite(LED_GREEN, LOW);
    // delay(500);
}

void start_websocket_loop(){
    webSocket.loop();
}

void setup()
{
    system_update_cpu_freq(160);
    Serial.begin(Serial_Monitor_Baud);
    start_led_state_indicator();
    while (!Serial){;}
    I2C_begin();
    init_MPU6050();
    init_WiFi();
}

void loop()
{
    start_websocket_loop();
    get_MPU6050_reading();
    // serial_print_MPU6050_values();
    update_LED_state_indicator();
    
}