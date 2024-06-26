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

    void ExecuteGCode(GCodeCommand* gc);

    void G0(GCodeCommand* gc);// Rapid move
    void G1(GCodeCommand* gc);// Linear move
    void G4(GCodeCommand* gc);// Dwell
    void G21(GCodeCommand* gc);// Millimeter mode
    void G28(GCodeCommand* gc);// Home all non-relative axes
    void G90(GCodeCommand* gc);// Absolute Positioning Mode
    void G91(GCodeCommand* gc);// Relative Positioning Mode
    void G92(GCodeCommand* gc);// Set global offsets

    void M17(GCodeCommand* gc);// Enable steppers
    void M18(GCodeCommand* gc);// Disable steppers
    void M42(GCodeCommand* gc);// Manual pin control
    void M114(GCodeCommand* gc);// Get Position
    void M115(GCodeCommand* gc);// Detect Firmware
    void M150(GCodeCommand* gc);// Set LED color
    void M204(GCodeCommand* gc);// Set Acceleration
    void M400(GCodeCommand* gc);// Wait for move completion
    
};

