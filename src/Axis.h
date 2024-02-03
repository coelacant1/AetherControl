#pragma once

#include "AxisLimits.h"

class Axis {
private:
    AxisLimits axisLimits;
    int stepPin;
    int dirPin;
    int enablePin;
    int endstopPin = -1;

    float stepsPerMillimeter;
    long currentPositionSteps;
    bool isHomed = false;

    float targetPosition = 0.0f;//mm
    float targetVelocity = 0.0f;//mm/s
    float targetAcceleration = 0.0f;//mm/s2
    
    float position;//mm
    float velocity;//mm/s
    float acceleration;//mm/s2

    void Step(long steps); // Executes a given number of steps
    bool ReadEndstop() const; // Reads the endstop state
    void AutoHome(); // Perform auto-homing procedure

public:
    Axis(int stepPin, int dirPin, int enablePin, float stepsPerMillimeter); // Relative axis
    Axis(int stepPin, int dirPin, int enablePin, int endstopPin, float stepsPerMillimeter); // Absolute axis

    void SetAxisLimits(AxisLimits axisLimits);

    void MoveTo(float position); // Moves to a position in mm
    void MoveBy(float distance); // Moves by a distance in mm
    void Home(); // Homes the axis
    void Enable(); // Enables the stepper motor
    void Disable(); // Disables the stepper motor

    void SetStepsPerMillimeter(float stepsPerMM); // Configures steps per mm



    float GetCurrentPositionMM() const; // Returns the current position in mm
    long GetCurrentPositionSteps() const; // Returns the current position in steps

    float GetCurrentVelocity() const;
    float GetCurrentAcceleration() const;

    void SetTargetPosition(float position){}
    void SetTargetVelocity(float velocity);
    void SetTargetAcceleration(float acceleration);

    void Update();//Updates the current axis to follow the command set

};
