#pragma once

#include "Axis.h"
#include "Cartesian.h"
#include "CoreXY.h"
#include "PathPlanner.h"
#include "PulseControl.h"
#include "WS2812B.h"

#include "Implementation.h"

class PickPlaceHead : public Implementation{
private:
    AxisConstraints axisLimitZ = AxisConstraints('Z', 0.0f, 48.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitI = AxisConstraints('I', -720.0f, 720.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);

    PulseControl<2> pulseControl = PulseControl<2>();
    Cartesian<2> cartesian = Cartesian<2>();

    Axis axisZ = Axis(&pulseControl, &axisLimitZ, 6, 7, 5, 8);
    Axis axisI = Axis(&pulseControl, &axisLimitI, 3, 4, 2);

    WS2812B<20> wsLEDs = WS2812B<20>(9);

public:
    PickPlaceHead() : Implementation(GCode(&cartesian)){}

    void Initialize() override;
    void PrintInformation() override;

};

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
