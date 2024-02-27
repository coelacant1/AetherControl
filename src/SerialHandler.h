#pragma once

#include <Arduino.h>
#include "GCodeCommand.h"

// Creates a GCode command object
class SerialHandler {
private:
    static HardwareSerial * serial;
    static long baudrate;

public:
    static void SetSerialInterface(HardwareSerial & serial, long baudrate);
    static void Initialize();
    static bool CommandAvailable();
    static GCodeCommand ReadCommand();
    static void SendOK();
    static void SendNotImplemented();
    static void SendMessage(String message);
    
	template <typename T>
    static void SendMessageValue(String message, T value);
};


HardwareSerial * SerialHandler::serial;
long SerialHandler::baudrate;

void SerialHandler::SetSerialInterface(HardwareSerial & serial, long baudrate){
    SerialHandler::serial = &serial;
    SerialHandler::baudrate = baudrate;
}


void SerialHandler::Initialize(){
    SerialHandler::serial->begin(baudrate);
}

bool SerialHandler::CommandAvailable(){
    return serial->available();
}

GCodeCommand SerialHandler::ReadCommand(){
    GCodeCommand cmd;
    String line;
    char incomingChar;

    while (true) {
        if(serial->available() > 0){
            incomingChar = serial->read();
            
            //serial->print(incomingChar);
            
            if (incomingChar == '\n') break;

            if (incomingChar == '\b') {
                if (line.length() > 0) {
                    line.remove(line.length() - 1);
                }
            }
            else {
                line += incomingChar;
            }
            
        }
    }
    
    // Extract the command type and number
    int firstSpaceIndex = line.indexOf(' ');
    if (firstSpaceIndex != -1) {
        String commandPart = line.substring(0, firstSpaceIndex);
        if (commandPart.length() > 1) {
            cmd.commandType = commandPart.charAt(0);
            cmd.commandNumber = commandPart.substring(1).toInt();
        }

        // Process parameters
        String parameters = line.substring(firstSpaceIndex + 1);
        int nextSpaceIndex;
        while ((nextSpaceIndex = parameters.indexOf(' ')) != -1) {
            if (cmd.parametersUsed >= GCodeCommand::parameterCount) {
                break; // Ensure we don't exceed the array bounds
            }
            
            String parameter = parameters.substring(0, nextSpaceIndex);
            if (parameter.length() > 1) {
                cmd.characters[cmd.parametersUsed] = parameter.charAt(0); // Store parameter character
                cmd.values[cmd.parametersUsed] = parameter.substring(1).toFloat(); // Convert and store value
                cmd.parametersUsed++; // Increment the count of parameters used
            }
            parameters = parameters.substring(nextSpaceIndex + 1);
        }

        // Catch any last parameter not followed by a space
        if (parameters.length() > 1 && cmd.parametersUsed < GCodeCommand::parameterCount) {
            cmd.characters[cmd.parametersUsed] = parameters.charAt(0);
            cmd.values[cmd.parametersUsed] = parameters.substring(1).toFloat();
            cmd.parametersUsed++; // Increment here as well
        }
    }
    else{
        if (line.length() > 1) {
            cmd.commandType = line.charAt(0);
            cmd.commandNumber = line.substring(1).toInt();
        }
    }

    return cmd;
}

void SerialHandler::SendOK(){
    serial->println("ok");
}

void SerialHandler::SendNotImplemented(){
    serial->println("error");
}

void SerialHandler::SendMessage(String message){
    serial->println(message);
}

template <typename T>
void SerialHandler::SendMessageValue(String message, T value){
    serial->print(message); serial->print(": ");
    serial->println(value);
}