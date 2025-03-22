#pragma once

#include "GCodeCommand.h"

class GCodeCommandList {
public:
    static constexpr long commandCount = 6;

    static const GCodeCommand commandList[commandCount];

    static const GCodeCommand* GetCommand(long index) {
        if (index >= 0 && index < commandCount)
            return &commandList[index];
        else
            return nullptr;
    }
};

const GCodeCommand GCodeCommandList::commandList[commandCount] = {
    GCodeCommand('G', 6, 'X', 200.0f, 'Y', 200.0f, 'Z', 200.0f, 'U', 200.0f, 'V', 200.0f, 'W', 200.0f, 'F', 6000.0f),
    GCodeCommand('G', 4, 'P', 250.0f),
    GCodeCommand('G', 6, 'X', 300.0f, 'Y', 300.0f, 'Z', 300.0f, 'U', 300.0f, 'V', 300.0f, 'W', 300.0f, 'F', 3000.0f),
    GCodeCommand('G', 4, 'P', 250.0f),
    GCodeCommand('G', 6, 'X', 100.0f, 'Y', 100.0f, 'Z', 100.0f, 'U', 100.0f, 'V', 100.0f, 'W', 100.0f, 'F', 9000.0f),
    GCodeCommand('G', 4, 'P', 250.0f)
};
