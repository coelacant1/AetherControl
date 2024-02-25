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
    GCodeCommand() : commandType('\0'), commandNumber(-1), parametersUsed(0) {
        // Initialize arrays to default values
        for (uint8_t i = 0; i < parameterCount; ++i) {
            characters[i] = '\0'; // Initialize to null character
            values[i] = 0.0f; // Initialize to zero
        }
    }
};
