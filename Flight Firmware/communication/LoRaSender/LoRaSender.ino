#include <SPI.h>
#include <LoRa.h>


  //LoR32u4II 868MHz or 915MHz (black board)
  #define SCK     15
  #define MISO    14
  #define MOSI    16
  #define SS      8
  #define RST     4
  #define DI0     7
  #define BAND    915E6  // 915E6 //868E6
  #define PABOOST true 

int counter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Sender");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND,PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);
  // send packet
  LoRa.beginPacket();
  LoRa.print(counter);
  LoRa.print(",12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345,12.345");
  LoRa.endPacket();
  counter++;
  delay(1000);
}
