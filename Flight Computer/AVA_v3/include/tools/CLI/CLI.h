#ifndef __cli_h__
#define __cli_h__

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

#endif