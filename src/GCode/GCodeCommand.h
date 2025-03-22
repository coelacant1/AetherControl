#pragma once

#include <Arduino.h>

class GCodeCommand {
private:
    void UnpackParams(uint8_t idx, char chars[], float vals[]);

    template<typename TChar, typename TVal, typename... ArgsRest>
    void UnpackParams(uint8_t idx, char chars[], float vals[], TChar c, TVal v, ArgsRest... rest);

public:
    static const uint8_t parameterCount = 12;
    char commandType; // 'G' for G-code commands, 'M' for machine commands, etc.
    int commandNumber; // The number part of the command, e.g., 0 in G0
    char characters[parameterCount];
    float values[parameterCount];
    uint8_t parametersUsed;

    // Constructor to initialize default values
    GCodeCommand();

    template<typename... Args>
    GCodeCommand(char type, int number, Args... args);

    String GetString() const;

};

#include "GCodeCommand.tpp"