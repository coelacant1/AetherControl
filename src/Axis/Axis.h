#pragma once

#include <Arduino.h>

#include "AxisConstraints.h"
#include "../Hardware/IPulseControl.h"
#include "../Serial/SerialHandler.h"

#include "../lib/ProtoTracer/Utils/Math/Mathematics.h"

class Axis {
private:
    static uint8_t instanceCount;
    uint8_t instanceNumber = 0;
    IPulseControl* pulseControl;
    AxisConstraints* axisConstraints;
    uint8_t stepPin = -1;
    uint8_t dirPin = -1;
    uint8_t enablePin = -1;
    uint8_t endstopPin = -1;

    bool homeMax = true;
    bool isHoming = false;
    bool isRelative = false;
    bool isEnabled = false;
    bool newCommand = true;
    bool invertDirection = false;

    float controlPosition = 0.0f;
    float previousTargetPosition = 0.0f;
    float previousControlPosition = 0.0f;

    float targetPosition = 0.0f;//mm
    float targetVelocity = 0.0f;//mm/s
    
    float position = 0.0f;//mm
    float velocity = 0.0f;//mm/s

    float lastTravelEstimate;
    elapsedMicros sinceTravelEstimate;
    elapsedMicros sinceUpdate;

public:
    Axis(IPulseControl* pulseControl, AxisConstraints* axisConstraints, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin); // Relative axis
    Axis(IPulseControl* pulseControl, AxisConstraints* axisConstraints, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t endstopPin); // Absolute axis

    void Initialize();

    void SetHomeDirection(bool towardsMax);

    void ResetRelative();// Relative equivalent of homing
    void AutoHome(); // Perform auto-homing procedure
    void InvertDirection(bool invert);// Invert axis direction
    void Enable(); // Enables the stepper motor
    void Disable(); // Disables the stepper motor

    bool IsRelative() const;
    bool ReadEndstop() const; // Reads the endstop state

    AxisConstraints* GetAxisConstraints();
    float GetCurrentPosition(); // Returns the current position in mm
    float GetCurrentVelocity() const;
    float GetTargetPosition() const;// 
    float GetPreviousTargetPosition() const;
    float GetTargetVelocity() const;// 

    void SetCurrentPosition(float position);
    void SetCurrentVelocity(float velocity);// Sets current control loop to velocity
    void SetTargetPosition(float targetPosition);// Returns constrained output
    void SetTargetVelocity(float targetVelocity);// Returns constrained output

    void Update();//Updates the current axis to follow the command set

};

