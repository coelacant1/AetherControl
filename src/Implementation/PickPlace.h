#pragma once

#include "Axis.h"
#include "CoreXY.h"
#include "PathPlanner.h"
#include "PulseControl.h"
#include "WS2812B.h"
#include "Implementation.h"

class PickPlace : public Implementation{
private:
    AxisConstraints axisLimitA = AxisConstraints('A', 0.0f, 1000.0f, 0.01f, 1000.0f, 50000.0f, 20000.0f, 160.0f);
    AxisConstraints axisLimitB = AxisConstraints('B', 0.0f, 1000.0f, 0.01f, 1000.0f, 50000.0f, 20000.0f, 160.0f);

    PulseControl<2> pulseControl = PulseControl<2>();
    CoreXY<2> coreXY = CoreXY<2>(18, 16, true, true, 300, 400);

    Axis axisA = Axis(&pulseControl, &axisLimitA, 32, 31, 30);
    Axis axisB = Axis(&pulseControl, &axisLimitB, 8, 7, 6);

    WS2812B<20> wsLEDs = WS2812B<20>(19);

public:
    PickPlace() : Implementation(GCode(&coreXY)){}

    void Initialize() override;
    void PrintInformation() override;

};

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
