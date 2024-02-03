#pragma once

#include "GCodeCommand.h"

class Controller {
private:
    static const int commandMemory = 50;
    static GCodeCommand nextCommands[commandMemory];
    static int commands;

public:
    static void AddCommand(GCodeCommand gCodeCommand);

    static void Update();
};

int Controller::commands = 0;
