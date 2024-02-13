#include <Arduino.h>

#include "Axis.h"
#include "PathPlanner.h"
#include "PulseControl.h"
#include "..\ProtoTracer\Utils\Debug.h"

AxisLimits axisLimitZ = AxisLimits(0.0f, 40.0f, 0.01f, 1000.0f, 1000000.0f);
AxisLimits axisLimitR = AxisLimits(-720.0f, 720.0f, 0.01f, 1000.0f, 1000000.0f);

Axis<2> axisZ = Axis<2>(6, 7, 5, 8, 80.0f, 100.0f);//8, 
Axis<2> axisR = Axis<2>(3, 4, 2, 80.0f, 100.0f);

PathPlanner<2> pathPlanner = PathPlanner<2>();

long increment = 0;

void setup() {
    //initialize hardware
    axisZ.SetAxisLimits(axisLimitZ);
    axisR.SetAxisLimits(axisLimitR);

    axisZ.Initialize();
    axisR.Initialize();

    axisZ.Enable();
    axisR.Enable();

    pathPlanner.AddAxis(&axisZ);
    pathPlanner.AddAxis(&axisR);

    axisZ.SetHomeDirection(true);

    PulseControl<2>::Enable();

    Serial.begin(250000);
    Serial.println("Starting...");
}

void printAxisData(){
    Serial.print(axisZ.GetCurrentVelocity()); Serial.print('\t');
    Serial.print(axisZ.GetCurrentPosition()); Serial.print('\t');
    Serial.print(axisZ.GetTargetPosition()); Serial.print('\t');
    Serial.print(axisZ.GetTargetVelocity()); Serial.print("\t\t");

    Serial.print(axisR.GetCurrentVelocity()); Serial.print('\t');
    Serial.print(axisR.GetCurrentPosition()); Serial.print('\t');
    Serial.print(axisR.GetTargetPosition()); Serial.print('\t');
    Serial.print(axisR.GetTargetVelocity()); Serial.print('\n');
}

void printTargetVPosition(){
    Serial.print("Target Z: "); Serial.print(axisZ.GetTargetPosition(), 5);
    Serial.print("\tPosition Z: "); Serial.print(axisZ.GetCurrentPosition(), 5);
    Serial.print("\tTarget R: "); Serial.print(axisR.GetTargetPosition(), 5);
    Serial.print("\tPosition Z: "); Serial.println(axisR.GetCurrentPosition(), 5);
}

void printTargetWVelocity(){
    Serial.print("Target Z: "); Serial.print(axisZ.GetTargetPosition(), 2);
    Serial.print("\tTarget R: "); Serial.print(axisR.GetTargetPosition(), 2);
    Serial.print("\tTarget Vel Z: "); Serial.print(axisZ.GetTargetVelocity(), 2);
    Serial.print("\tTarget Vel R: "); Serial.println(axisR.GetTargetVelocity(), 2);
}

void loop() {// 10:28
    Serial.print(Debug::FreeMem()); Serial.print('\t');
    Serial.print(increment / 3600); Serial.print('\t');
    Serial.print(increment / 60); Serial.print('\t');
    Serial.print(increment % 60); Serial.print('\t');
    Serial.println(increment++);
    /*
    Serial.println("Homing Z");
    axisZ.AutoHome();

    delay(1000);
    
    Serial.println("Moving Z30, R60");
    axisZ.SetTargetPosition(30.0f);
    axisR.SetTargetPosition(60.0f);

    pathPlanner.CalculateLimits(100.0f);

    while (pathPlanner.Update()) delay(50);

    delay(1000);
    
    Serial.println("Moving Z0, R0");
    axisZ.SetTargetPosition(0.0f);
    axisR.SetTargetPosition(0.0f);
    
    pathPlanner.CalculateLimits(100.0f);

    while (pathPlanner.Update()) delay(50);

    */
    delay(1000);
}
