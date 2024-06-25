#include "PickPlace.h"

void PickPlace::Initialize(){
    //initialize hardware
    axisA.Initialize();
    axisB.Initialize();

    axisA.Disable();
    axisB.Disable();

    coreXY.AddAxis(&axisA, IKinematics::A);
    coreXY.AddAxis(&axisB, IKinematics::B);
    
    //axisA.SetHomeDirection(true);
    //axisB.SetHomeDirection(true);
    
    wsLEDs.Initialize();
    wsLEDs.SetColor(0, 0, 0);
    wsLEDs.Update();

    pulseControl.Initialize();
    pulseControl.Enable();

    axisA.InvertDirection(true);
    axisB.InvertDirection(true);

    gCode.AddLEDs(&wsLEDs);
}

void PickPlace::PrintInformation(){
    SerialHandler::SendMessageTab(axisLimitA.GetAxisLabel());
    SerialHandler::SendMessageTab(axisLimitA.GetAcceleration());
    SerialHandler::SendMessageTab(axisLimitA.GetMaxAcceleration());
    SerialHandler::SendMessageTab(axisLimitA.GetMaxPosition());
    SerialHandler::SendMessageTab(axisLimitA.GetMaxVelocity());
    SerialHandler::SendMessageTab(axisLimitA.GetMinPosition());
    SerialHandler::SendMessageTab(axisLimitA.GetMinVelocity());
    SerialHandler::SendMessageTab(axisLimitA.GetMinVelocity());
    SerialHandler::SendMessage(axisLimitA.GetStepsPerMillimeter());
    
    SerialHandler::SendMessageTab(axisLimitB.GetAxisLabel());
    SerialHandler::SendMessageTab(axisLimitB.GetAcceleration());
    SerialHandler::SendMessageTab(axisLimitB.GetMaxAcceleration());
    SerialHandler::SendMessageTab(axisLimitB.GetMaxPosition());
    SerialHandler::SendMessageTab(axisLimitB.GetMaxVelocity());
    SerialHandler::SendMessageTab(axisLimitB.GetMinPosition());
    SerialHandler::SendMessageTab(axisLimitB.GetMinVelocity());
    SerialHandler::SendMessageTab(axisLimitB.GetMinVelocity());
    SerialHandler::SendMessage(axisLimitB.GetStepsPerMillimeter());
}
