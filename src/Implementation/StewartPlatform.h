#pragma once

#include "Axis.h"
#include "Cartesian.h"
#include "PathPlanner.h"
#include "PulseControl.h"

#include "Implementation.h"

class StewartPlatform : public Implementation{
private:
    AxisConstraints axisLimitX = AxisConstraints('X', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitY = AxisConstraints('Y', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitZ = AxisConstraints('Z', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitU = AxisConstraints('U', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitV = AxisConstraints('V', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitW = AxisConstraints('W', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);

    PulseControl<6> pulseControl = PulseControl<6>();
    Cartesian<6> cartesian = Cartesian<6>();

    Axis axisX = Axis(&pulseControl, &axisLimitX, 6, 7, 5);
    Axis axisY = Axis(&pulseControl, &axisLimitY, 6, 7, 5);
    Axis axisZ = Axis(&pulseControl, &axisLimitZ, 6, 7, 5);
    Axis axisU = Axis(&pulseControl, &axisLimitU, 6, 7, 5);
    Axis axisV = Axis(&pulseControl, &axisLimitV, 6, 7, 5);
    Axis axisW = Axis(&pulseControl, &axisLimitW, 6, 7, 5);

public:
    StewartPlatform():Implementation(GCode(&cartesian)){}

    void Initialize() override;
    void PrintInformation() override;

};

void StewartPlatform::Initialize(){
    Axis* axes[6] = { &axisX, &axisY, &axisZ, &axisU, &axisV, &axisW };

    //initialize hardware
    for (int i = 0; i < 6; i++){
        axes[i]->Initialize();
        axes[i]->Disable();
        axes[i]->ResetRelative();
    }

    cartesian.AddAxis(&axisX, IKinematics::X);
    cartesian.AddAxis(&axisY, IKinematics::Y);
    cartesian.AddAxis(&axisZ, IKinematics::Z);
    cartesian.AddAxis(&axisU, IKinematics::U);
    cartesian.AddAxis(&axisV, IKinematics::V);
    cartesian.AddAxis(&axisW, IKinematics::W);

    pulseControl.Initialize();
    pulseControl.Enable();
}

void StewartPlatform::PrintInformation(){
    AxisConstraints* axesC[6] = { &axisLimitX, &axisLimitY, &axisLimitZ, &axisLimitU, &axisLimitV, &axisLimitW };

    SerialHandler::SendMessageTab("Label");
    SerialHandler::SendMessageTab("Accel");
    SerialHandler::SendMessageTab("Max Accel");
    SerialHandler::SendMessageTab("Max Pos");
    SerialHandler::SendMessageTab("Max Vel");
    SerialHandler::SendMessageTab("Min Pos");
    SerialHandler::SendMessageTab("MinVel");
    SerialHandler::SendMessage("Steps per MM");

    for (int i = 0; i < 6; i++){
        SerialHandler::SendMessageTab(axesC[i]->GetAxisLabel());
        SerialHandler::SendMessageTab(axesC[i]->GetAcceleration());
        SerialHandler::SendMessageTab(axesC[i]->GetMaxAcceleration());
        SerialHandler::SendMessageTab(axesC[i]->GetMaxPosition());
        SerialHandler::SendMessageTab(axesC[i]->GetMaxVelocity());
        SerialHandler::SendMessageTab(axesC[i]->GetMinPosition());
        SerialHandler::SendMessageTab(axesC[i]->GetMinVelocity());
        SerialHandler::SendMessage(axesC[i]->GetStepsPerMillimeter());
    }
}
