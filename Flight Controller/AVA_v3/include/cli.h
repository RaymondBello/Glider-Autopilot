#ifndef __CLI_H__
#define __CLI_H__

#include <Arduino.h>
#include <SimpleCLI.h>

class CliCommand {

    private:
        SimpleCLI cli;
        Command cmdGet;
        Command cmdSet;
        Command cmdExec;
  
    public:
        CliCommand();
        void handleReceivedMessage(char* msg);
        void handleSerial();
        void processGetCommand(const char*);
        void processSetCommand(const char* setting, const char* value);
};

#endif // _CLI_H_