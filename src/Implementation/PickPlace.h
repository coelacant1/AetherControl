#pragma once

#include "Axis.h"
#include "CoreXY.h"
#include "PathPlanner.h"
#include "PulseControl.h"
#include "WS2812B.h"
#include "Implementation.h"

class PickPlace : public Implementation{
private:
    AxisConstraints axisLimitZ = AxisConstraints('Z', 0.0f, 48.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitI = AxisConstraints('I', -720.0f, 720.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);

    PulseControl<2> pulseControl = PulseControl<2>();
    CoreXY<2> coreXY = CoreXY<2>(0, 1, true, true, 300, 400);

    Axis axisZ = Axis(&pulseControl, &axisLimitZ, 6, 7, 5, 8);
    Axis axisI = Axis(&pulseControl, &axisLimitI, 3, 4, 2);

    WS2812B<20> wsLEDs = WS2812B<20>(9);

public:
    PickPlace() : Implementation(GCode(&coreXY)){}

    void Initialize() override;
    void PrintInformation() override;

};

void PickPlace::Initialize(){
    //initialize hardware
    axisZ.Initialize();
    axisI.Initialize();

    axisZ.Disable();
    axisI.Disable();

    coreXY.AddAxis(&axisZ, IKinematics::Z);
    coreXY.AddAxis(&axisI, IKinematics::I);
    
    axisZ.SetHomeDirection(true);
    
    wsLEDs.Initialize();
    wsLEDs.SetColor(0, 0, 0);
    wsLEDs.Update();

    pulseControl.Initialize();
    pulseControl.Enable();

    gCode.AddLEDs(&wsLEDs);
}

void PickPlace::PrintInformation(){
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
