#include <Arduino.h>

#include "Axis.h"
#include "PathPlanner.h"
#include "PulseControl.h"

AxisLimits axisLimitZ = AxisLimits(0.0f, 400.0f, 0.1f, 1000.0f, 1000000.0f);
AxisLimits axisLimitR = AxisLimits(0.0f, 400.0f, 0.1f, 1000.0f, 1000000.0f);

Axis<2> axisZ = Axis<2>(3, 4, 2, 80.0f, 100.0f);//8, 
Axis<2> axisR = Axis<2>(6, 7, 5, 80.0f, 100.0f);

PathPlanner<2> pathPlanner = PathPlanner<2>();

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

    PulseControl<2>::Enable();

    Serial.begin(250000);
    Serial.println("Starting...");
}

void printAxisData(){
    Serial.print(axisZ.GetCurrentVelocity()); Serial.print('\t');
    Serial.print(axisZ.GetCurrentPositionMM()); Serial.print('\t');
    Serial.print(axisZ.GetTargetPosition()); Serial.print('\t');
    Serial.print(axisZ.GetTargetVelocity()); Serial.print("\t\t");

    Serial.print(axisR.GetCurrentVelocity()); Serial.print('\t');
    Serial.print(axisR.GetCurrentPositionMM()); Serial.print('\t');
    Serial.print(axisR.GetTargetPosition()); Serial.print('\t');
    Serial.print(axisR.GetTargetVelocity()); Serial.print('\n');
}

void printTargetVPosition(){
    Serial.print("Target Z: "); Serial.print(axisZ.GetTargetPosition(), 5);
    Serial.print("\tPosition Z: "); Serial.print(axisZ.GetCurrentPositionMM(), 5);
    Serial.print("\tTarget R: "); Serial.print(axisR.GetTargetPosition(), 5);
    Serial.print("\tPosition Z: "); Serial.println(axisR.GetCurrentPositionMM(), 5);
}

void printTargetWVelocity(){
    Serial.print("Target Z: "); Serial.print(axisZ.GetTargetPosition(), 2);
    Serial.print("\tTarget R: "); Serial.print(axisR.GetTargetPosition(), 2);
    Serial.print("\tTarget Vel Z: "); Serial.print(axisZ.GetTargetVelocity(), 2);
    Serial.print("\tTarget Vel R: "); Serial.println(axisR.GetTargetVelocity(), 2);
}

void loop() {
    //update controller
    axisZ.SetTargetPosition(400.0f);
    axisR.SetTargetPosition(300.0f);

    pathPlanner.Update(100.0f);
    printTargetWVelocity();

    while (!Mathematics::IsClose(axisZ.GetCurrentPositionMM(), axisZ.GetTargetPosition(), 0.01f) || !Mathematics::IsClose(axisR.GetCurrentPositionMM(), axisR.GetTargetPosition(), 0.01f)) {
        axisZ.Update();
        axisR.Update();

        //printAxisData();
        Serial.print('\n');

        delay(1);
    }

    printTargetVPosition();

    delay(1000);

    axisZ.SetTargetPosition(0.0f);
    axisR.SetTargetPosition(100.0f);
    
    pathPlanner.Update(100.0f);
    printTargetWVelocity();

    while (!Mathematics::IsClose(axisZ.GetCurrentPositionMM(), axisZ.GetTargetPosition(), 0.01f) || !Mathematics::IsClose(axisR.GetCurrentPositionMM(), axisR.GetTargetPosition(), 0.01f)) {
        axisZ.Update();
        axisR.Update();

        //printAxisData();
        Serial.print('\n');

        delay(1);
    }

    printTargetVPosition();

    delay(1000);
}
