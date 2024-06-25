#include "StewartPlatform.h"

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
