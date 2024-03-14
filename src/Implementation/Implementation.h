#pragma once

#include <Arduino.h>

#include "GCode.h"
#include "SerialHandler.h"

class Implementation {
protected:
    GCode gCode;

public:
    Implementation(GCode gCode) : gCode(gCode){}

    virtual void Initialize() = 0;
    virtual void PrintInformation() = 0;

    void SetSerialInterface(HardwareSerial* serial, long baudrate);
    void SetSerialInterface(usb_serial_class* serial, long baudrate);

    bool IsCommandAvailable();
    void ExecuteCommand();

};

void Implementation::SetSerialInterface(HardwareSerial* serial, long baudrate){
    SerialHandler::SetSerialInterface(Serial, 38400);
    SerialHandler::Initialize();
}

void Implementation::SetSerialInterface(usb_serial_class* serial, long baudrate){
    SerialHandler::SetSerialInterface(Serial, 38400);
    SerialHandler::Initialize();
}

bool Implementation::IsCommandAvailable(){
    return SerialHandler::CommandAvailable();
}

void Implementation::ExecuteCommand(){
    GCodeCommand cmd = SerialHandler::ReadCommand();

    gCode.ExecuteGCode(&cmd);

    SerialHandler::SendOK();

    SerialHandler::SendCommandAsk();
}
