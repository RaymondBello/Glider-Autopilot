// macros from DateTime.h
/* Useful Constants */
#define SECS_PER_MIN (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) ((_time_ % SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) (_time_ / SECS_PER_DAY)

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

const int chipSelect = 4;

SoftwareSerial gpsSerial(10, 9); // RX, TX

String printDigits(byte digits, String packet)
{
  // utility function for digital clock display: prints colon and leading 0
  packet += String(":");
  if (digits < 10)
    packet += String("0");
  packet += String(digits);

  return packet;
}

String time(long val)
{
  String dataString = "";

  int days = elapsedDays(val);
  int hours = numberOfHours(val);
  int minutes = numberOfMinutes(val);
  int seconds = numberOfSeconds(val);

  dataString += String(days);

  dataString = printDigits(days,dataString);
  dataString = printDigits(hours, dataString);
  dataString = printDigits(minutes, dataString);
  dataString = printDigits(seconds, dataString);

  // digital clock display of current time
  return dataString;
}


void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  gpsSerial.begin(9600);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1)
      ;
  }
  Serial.println("card initialized.");
}

void loop()
{

  // unsigned long now_ms = millis();
  
  // // open the file. note that only one file can be open at a time,
  // // so you have to close this one before opening another.
  // File dataFile = SD.open("AVA-LOG.txt", FILE_WRITE);
  
  // // if the file is available, write to it:
  // if (dataFile)
  // {
  //   // dataFile.println(time(now_ms / 1000));
  //   dataFile.close();
  //   // print to the serial port too:
  //   Serial.println(time(now_ms / 1000));
  // }
  // // if the file isn't open, pop up an error:
  // else
  // {
  //   Serial.println("error opening datalog.txt");
  // }

  // delay(200);
  // Serial.print(gpsSerial.read());

  if (gpsSerial.available())
  {
    Serial.print(gpsSerial.read());
  }
}
