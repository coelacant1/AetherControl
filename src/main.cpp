#include <Arduino.h>

#include "Axis.h"
#include "GCode.h"
#include "PathPlanner.h"
#include "PulseControl.h"
#include "SerialHandler.h"
#include "WS2812B.h"
#include "..\ProtoTracer\Utils\Debug.h"

AxisConstraints axisLimitZ = AxisConstraints('Z', 0.0f, 48.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
AxisConstraints axisLimitI = AxisConstraints('I', -720.0f, 720.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);

PathPlanner<2> pathPlanner = PathPlanner<2>();
PulseControl<2> pulseControl = PulseControl<2>();

GCode gCode = GCode(&pathPlanner);

Axis axisZ = Axis(&pulseControl, &axisLimitZ, 6, 7, 5, 8);//8, 
Axis axisI = Axis(&pulseControl, &axisLimitI, 3, 4, 2);

WS2812B<5> wsLEDs = WS2812B<5>(9);

void setup() {
    //initialize hardware
    axisZ.Initialize();
    axisI.Initialize();

    axisZ.Enable();
    axisI.Enable();

    pathPlanner.AddAxis(&axisZ);
    pathPlanner.AddAxis(&axisI);

    axisZ.SetHomeDirection(true);
    
    wsLEDs.Initialize();
    wsLEDs.SetColor(0, 0, 0);
    wsLEDs.Update();

    pulseControl.Initialize();
    pulseControl.Enable();

    gCode.AddLEDs(&wsLEDs);

    SerialHandler::SetSerialInterface(Serial, 115200);
    SerialHandler::Initialize();

    
    Serial.print("Z: "); Serial.print('\t');
    Serial.print(axisLimitZ.GetAcceleration()); Serial.print('\t');
    Serial.print(axisLimitZ.GetAxisLabel()); Serial.print('\t');
    Serial.print(axisLimitZ.GetMaxAcceleration()); Serial.print('\t');
    Serial.print(axisLimitZ.GetMaxPosition()); Serial.print('\t');
    Serial.print(axisLimitZ.GetMaxVelocity()); Serial.print('\t');
    Serial.print(axisLimitZ.GetMinPosition()); Serial.print('\t');
    Serial.print(axisLimitZ.GetMinVelocity()); Serial.print('\t');
    Serial.print(axisLimitZ.GetStepsPerMillimeter()); Serial.print('\n');

    Serial.print("I: "); Serial.print('\t');
    Serial.print(axisLimitI.GetAcceleration()); Serial.print('\t');
    Serial.print(axisLimitI.GetAxisLabel()); Serial.print('\t');
    Serial.print(axisLimitI.GetMaxAcceleration()); Serial.print('\t');
    Serial.print(axisLimitI.GetMaxPosition()); Serial.print('\t');
    Serial.print(axisLimitI.GetMaxVelocity()); Serial.print('\t');
    Serial.print(axisLimitI.GetMinPosition()); Serial.print('\t');
    Serial.print(axisLimitI.GetMinVelocity()); Serial.print('\t');
    Serial.print(axisLimitI.GetStepsPerMillimeter()); Serial.print('\n');
}

void loop() {
    if (SerialHandler::CommandAvailable()){
        GCodeCommand cmd = SerialHandler::ReadCommand();

        gCode.ExecuteGCode(&cmd);

        SerialHandler::SendOK();

        delay(1);
    }
}
