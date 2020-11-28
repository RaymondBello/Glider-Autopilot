#include<Arduino.h>
#include "CLI.h"

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  bool received = getCommandLineFromSerialPort(CommandLine); //global CommandLine is defined in CommandLine.h
  if (received)
    DoMyCommand(CommandLine);
}