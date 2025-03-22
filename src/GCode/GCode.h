#pragma once

#include "../Axis/Axis.h"
#include "GCodeCommand.h"
#include "../Serial/SerialHandler.h"
#include "../Hardware/IWS2812B.h"
#include "../Kinematics/IKinematics.h"

#include "../GlobalVariables.h"

// Class for implementation of function calls for gcode commands
class GCode {
private:
    IWS2812B* leds;
    IKinematics* kinematics;

public:
    GCode(IKinematics* kinematics);

    void AddLEDs(IWS2812B* leds);
    void AddIKinematics(IKinematics* kinematics);

    void ExecuteGCode(const GCodeCommand* gc);

    void G0(const GCodeCommand* gc);// Rapid move
    void G1(const GCodeCommand* gc);// Linear move
    void G4(const GCodeCommand* gc);// Dwell
    void G6(const GCodeCommand* gc);// Direct Stepper Move, no acceleration
    void G21(const GCodeCommand* gc);// Millimeter mode
    void G28(const GCodeCommand* gc);// Home all non-relative axes
    void G90(const GCodeCommand* gc);// Absolute Positioning Mode
    void G91(const GCodeCommand* gc);// Relative Positioning Mode
    void G92(const GCodeCommand* gc);// Set global offsets

    void M17(const GCodeCommand* gc);// Enable steppers
    void M18(const GCodeCommand* gc);// Disable steppers
    void M42(const GCodeCommand* gc);// Manual pin control
    void M114(const GCodeCommand* gc);// Get Position
    void M115(const GCodeCommand* gc);// Detect Firmware
    void M150(const GCodeCommand* gc);// Set LED color
    void M204(const GCodeCommand* gc);// Set Acceleration
    void M400(const GCodeCommand* gc);// Wait for move completion
    
};

