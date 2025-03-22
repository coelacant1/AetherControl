#pragma once

#include <Arduino.h>

#include "../GCode/GCode.h"
#include "../Serial/SerialHandler.h"

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
    void DirectExecuteCommand(String command);

    GCode* GetGCodeControl();

};
