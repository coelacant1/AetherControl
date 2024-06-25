#include "PickPlaceHead.h"

void PickPlaceHead::Initialize(){
    //initialize hardware
    axisZ.Initialize();
    axisI.Initialize();

    axisZ.Disable();
    axisI.Disable();

    cartesian.AddAxis(&axisZ, IKinematics::Z);
    cartesian.AddAxis(&axisI, IKinematics::I);
    
    axisZ.SetHomeDirection(true);
    
    wsLEDs.Initialize();
    wsLEDs.SetColor(0, 0, 0);
    wsLEDs.Update();

    pulseControl.Initialize();
    pulseControl.Enable();

    gCode.AddLEDs(&wsLEDs);
}

void PickPlaceHead::PrintInformation(){
    SerialHandler::SendMessageTab(axisLimitZ.GetAxisLabel());
    SerialHandler::SendMessageTab(axisLimitZ.GetAcceleration());
    SerialHandler::SendMessageTab(axisLimitZ.GetMaxAcceleration());
    SerialHandler::SendMessageTab(axisLimitZ.GetMaxPosition());
    SerialHandler::SendMessageTab(axisLimitZ.GetMaxVelocity());
    SerialHandler::SendMessageTab(axisLimitZ.GetMinPosition());
    SerialHandler::SendMessageTab(axisLimitZ.GetMinVelocity());
    SerialHandler::SendMessageTab(axisLimitZ.GetMinVelocity());
    SerialHandler::SendMessage(axisLimitZ.GetStepsPerMillimeter());
    
    SerialHandler::SendMessageTab(axisLimitI.GetAxisLabel());
    SerialHandler::SendMessageTab(axisLimitI.GetAcceleration());
    SerialHandler::SendMessageTab(axisLimitI.GetMaxAcceleration());
    SerialHandler::SendMessageTab(axisLimitI.GetMaxPosition());
    SerialHandler::SendMessageTab(axisLimitI.GetMaxVelocity());
    SerialHandler::SendMessageTab(axisLimitI.GetMinPosition());
    SerialHandler::SendMessageTab(axisLimitI.GetMinVelocity());
    SerialHandler::SendMessageTab(axisLimitI.GetMinVelocity());
    SerialHandler::SendMessage(axisLimitI.GetStepsPerMillimeter());
}
