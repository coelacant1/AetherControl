#pragma once

#include "../Axis/Axis.h"
#include "GCodeCommand.h"
#include "../Serial/SerialHandler.h"
#include "../Hardware/IWS2812B.h"
#include "../Kinematics/IKinematics.h"

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
    void G28(GCodeCommand* gc);// Home all non-relative axes

    void M17(GCodeCommand* gc);// Enable steppers
    void M18(GCodeCommand* gc);// Disable steppers
    void M42(GCodeCommand* gc);// Manual pin control
    void M150(GCodeCommand* gc);// Set LED color
    
};

