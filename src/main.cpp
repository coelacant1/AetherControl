#include <Arduino.h>

#include "Axis.h"
#include "PathPlanner.h"
#include "PulseControl.h"
#include "WS2812B.h"
#include "..\ProtoTracer\Utils\Debug.h"

AxisLimits axisLimitZ = AxisLimits(0.0f, 48.0f, 0.01f, 1000.0f, 10000.0f);
AxisLimits axisLimitR = AxisLimits(-720.0f, 720.0f, 0.01f, 1000.0f, 10000.0f);

Axis<2> axisZ = Axis<2>(6, 7, 5, 8, 80.0f, 1200.0f);//8, 
Axis<2> axisR = Axis<2>(3, 4, 2, 80.0f, 1200.0f);

PathPlanner<2> pathPlanner = PathPlanner<2>();

WS2812B<5> wsLEDs = WS2812B<5>(9);

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
    
    wsLEDs.Initialize();
    wsLEDs.SetColor(10, 10, 10);
    wsLEDs.Update();

    PulseControl<2>::Initialize();
    PulseControl<2>::Enable();


    Serial.begin(250000);
    Serial.println("Starting...");
}

void loop() {
    Serial.println("Homing Z");
    axisZ.AutoHome();
    axisR.ResetRelative();

    wsLEDs.SetColor(10, 10, 10);
    wsLEDs.Update();

    delay(1000);
    
    Serial.println("Moving Z30, R60");
    axisZ.SetTargetPosition(30.0f);
    axisR.SetTargetPosition(60.0f);

    pathPlanner.CalculateLimits(300.0f);

    while (pathPlanner.Update()) delay(10);


    wsLEDs.SetColor(10, 0, 0);
    wsLEDs.Update();

    delay(1000);
    
    Serial.println("Moving Z0, R0");
    axisZ.SetTargetPosition(0.0f);
    axisR.SetTargetPosition(0.0f);
    
    pathPlanner.CalculateLimits(800.0f);

    while (pathPlanner.Update()) delay(10);

    wsLEDs.SetColor(0, 10, 0);
    wsLEDs.Update();

    delay(1000);
    
    
    Serial.println("Moving Z15, R-60");
    axisZ.SetTargetPosition(15.0f);
    axisR.SetTargetPosition(-60.0f);
    
    pathPlanner.CalculateLimits(100.0f);

    while (pathPlanner.Update()) delay(50);

    wsLEDs.SetColor(0, 0, 10);
    wsLEDs.Update();

    delay(1000);
}
