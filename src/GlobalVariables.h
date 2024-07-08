#pragma once

#include <Arduino.h>

class GlobalVariables{
public:
    static bool relativeMode;

    static const bool printCharacterStream;

    static const float homingSpeedPrimary;
    static const float homingSpeedSecondary;
    static const float homingDistanceOffset;// Distance past axis length for finding the endstop
    static const float homingDistanceBackoff;

    static const char firmwareVersion[6];
    static const char firmwareName[14];
    static const char firmwareSourceURL[44];

};
