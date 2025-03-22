#include "GCodeCommand.h"

GCodeCommand::GCodeCommand() : commandType('\0'), commandNumber(-1), parametersUsed(0) {
    // Initialize arrays to default values
    for (uint8_t i = 0; i < parameterCount; ++i) {
        characters[i] = '\0'; // Initialize to null character
        values[i] = 0.0f; // Initialize to zero
    }
}

String GCodeCommand::GetString() const {
    String cmd;
    cmd += commandType;
    cmd += commandNumber;
    
    for (uint8_t i = 0; i < parametersUsed; i++) {
        cmd += ' ';
        cmd += characters[i];
        cmd += String(values[i], 4);
    }

    return cmd;
}

void GCodeCommand::UnpackParams(uint8_t idx, char chars[], float vals[]) {}
