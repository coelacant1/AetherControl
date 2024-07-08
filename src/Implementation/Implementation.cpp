#include "Implementation.h"

void Implementation::SetSerialInterface(HardwareSerial* serial, long baudrate){
    SerialHandler::SetSerialInterface(*serial, baudrate);
    SerialHandler::Initialize();
}

void Implementation::SetSerialInterface(usb_serial_class* serial, long baudrate){
    SerialHandler::SetSerialInterface(*serial, baudrate);
    SerialHandler::Initialize();
}

bool Implementation::IsCommandAvailable(){
    return SerialHandler::CommandAvailable();
}

void Implementation::ExecuteCommand(){
    GCodeCommand cmd = SerialHandler::ReadCommand();

    gCode.ExecuteGCode(&cmd);

    SerialHandler::SendOK();

    //SerialHandler::SendCommandAsk();
}
