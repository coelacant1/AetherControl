#pragma once

#include "GCodeCommand.h"

class GCodeCommandList {
public:
    static constexpr long commandCount = 1257;

    static const GCodeCommand commandList[commandCount];

    static const GCodeCommand* GetCommand(long index) {
        if (index >= 0 && index < commandCount)
            return &commandList[index];
        else
            return nullptr;
    }
};

const GCodeCommand GCodeCommandList::commandList[commandCount] = {

};
