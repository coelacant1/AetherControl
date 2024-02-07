#pragma once

struct GCodeCommand {
    char commandType; // 'G' for G-code commands, 'M' for machine commands, etc.
    int commandNumber; // The number part of the command, e.g., 0 in G0
    float X; // X coordinate, if applicable
    float Y; // Y coordinate, if applicable
    float Z; // Z coordinate, if applicable
    float A; // A coordinate, if applicable
    float B; // B coordinate, if applicable
    float C; // C coordinate, if applicable
    bool hasX; // Flag to indicate if X coordinate is provided
    bool hasY; // Flag to indicate if Y coordinate is provided
    bool hasZ; // Flag to indicate if Z coordinate is provided
    bool hasA; // Flag to indicate if Z coordinate is provided
    bool hasB; // Flag to indicate if Z coordinate is provided
    bool hasC; // Flag to indicate if Z coordinate is provided
    float feedrate; // Feedrate

    // Constructor to initialize default values
    GCodeCommand() : commandType('\0'), commandNumber(-1), X(0), Y(0), Z(0), A(0), B(0), C(0), hasX(false), hasY(false), hasZ(false), hasA(false), hasB(false), hasC(false), feedrate(0) {}
};
