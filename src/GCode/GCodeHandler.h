#pragma once

#include <Arduino.h>
#include "GCodeCommand.h"
#include "GCode.h"

// Creates a GCode command object
class GCodeHandler {
private:
    HardwareSerial* serial;
    long baudrate;

public:
    void SetSerialInterface(HardwareSerial& serial, long baudrate);
    void Initialize();
    GCodeCommand ReadCommand();
    void SendOK();
};
