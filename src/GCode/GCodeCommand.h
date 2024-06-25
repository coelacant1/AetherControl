#pragma once

#include <Arduino.h>

struct GCodeCommand {
    static const uint8_t parameterCount = 12;
    char commandType; // 'G' for G-code commands, 'M' for machine commands, etc.
    int commandNumber; // The number part of the command, e.g., 0 in G0
    char characters[parameterCount];
    float values[parameterCount];
    uint8_t parametersUsed;

    // Constructor to initialize default values
    GCodeCommand();
};
