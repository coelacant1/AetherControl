#pragma once

#include <Arduino.h>
#include "GCodeCommand.h"

// Creates a GCode command object
class GCodeParser {
private:
    // Helper methods to extract specific parts of the G-code line
    void ParseCommandTypeAndNumber(const String& part, GCodeCommand& cmd);
    void ParseCoordinate(const String& part, GCodeCommand& cmd);

public:
    // Method to parse a line of G-code into a GCodeCommand object
    GCodeCommand Parse(String line);
};
