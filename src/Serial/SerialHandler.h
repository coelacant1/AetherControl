#pragma once

#include <Arduino.h>
#include "../GCode/GCodeCommand.h"

// Creates a GCode command object
class SerialHandler {
private:
    static HardwareSerial* serialHS;
    static usb_serial_class* serialUSC;
    static long baudrate;
    static bool serialType;

public:
    static void Initialize();
    static bool IsReady();
    static int CommandAvailable();
    static GCodeCommand ReadCommand();
    static void SendOK();
    static void SendNotImplemented();
    static void SendCommandAsk();
    static void SendCharacter(char character);
    static void SendMessage(String message);
    static void SendMessageNLN(String message);
    static void SendMessageTab(String message);
    static void SendMessageSpace(String message);
    
    static void SetSerialInterface(HardwareSerial& serial, long baudrate);
    static void SetSerialInterface(usb_serial_class& serial, long baudrate);
    
	template <typename T>
    static void SendMessageValue(String message, T value);
    
	template <typename T>
    static void SendMessageValueSpace(String message, T value);
    
	template <typename T>
    static void SendMessageValues(String message, uint8_t count, T* value);
};

#include "SerialHandler.tpp"