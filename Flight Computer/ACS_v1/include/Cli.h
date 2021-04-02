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


void CliCommand::handleSerial() 
{
   const int BUFF_SIZE = 100; // make it big enough to hold your longest command
   static char buffer[BUFF_SIZE+1]; // +1 allows space for the null terminator
   static int length = 0; // number of characters currently in the buffer
//    static boolean newData = false;

    //    if(Serial.available())
    // while (Serial.available() > 0 && newData == false) {
    while (Serial.available() > 0 ) {
        char c = Serial.read();
        if((c == '\r') || (c == '\n')){
        // if(c == ';') {
            // end-of-line received
            // Serial.println("end-of-line received");
            
            if(length > 0) {
                this->handleReceivedMessage(buffer);
                
            }
            length = 0;
            // newData = true;
        } else {
            if(length < BUFF_SIZE) {
                buffer[length++] = c; // append the received character to the array
                buffer[length] = 0; // append the null terminator
                // Serial.print("Caractere ajouté : "); Serial.println(c);
            } else {
                // buffer full - discard the received character
                Serial.println("buffer full");
                
            }
        }
    }
    // Serial.println("End of handleSerial() function...");
}

void CliCommand::handleReceivedMessage(char* msg)
{
    String str(msg);

    this->cli.parse(str.c_str());

    // Check if a new error occurred
    if(this->cli.errored()) {
    CommandError e = this->cli.getError();

    // Print the error, or do whatever you want with it
    // Serial.println(e.toString());
    Serial.printf("\t%s\n", e.toString().c_str());
    }

    // First check if a newly parsed command is available
    if(this->cli.available()) {
        // Serial.print("It appears to be a valid command!");

        // Get the command out of the queue
        Command cmd = this->cli.getCommand();

        // Get the Argument(s) you want
        Argument argSetting = cmd.getArgument("setting"); // via name
        Argument argValue = cmd.getArgument("value"); // via index
        // strcpy(setting, argSetting.getValue());
        // strcpy(value, argValue.getValue());
        String setting = argSetting.getValue();
        String value = argValue.getValue();


        // Check if it's the command you're looking for
        if(cmd == this->cmdGet) {
            this->processGetCommand(setting.c_str());
        } 
        if(cmd == this->cmdSet) {
            // this->processSetCommand(setting.c_str(), value.c_str());
            Serial.printf("\tSetCmd: %s | Val: %s", setting.c_str());
            Serial.println();
        }
    } 
}




#endif // _CLI_H_