#pragma once

template<typename... Args>
GCodeCommand::GCodeCommand(char type, int number, Args... args) : commandType(type), commandNumber(number), parametersUsed(sizeof...(args) / 2) {
    // Initialize arrays to default values
    for (uint8_t i = 0; i < parameterCount; ++i) {
        characters[i] = '\0'; // Initialize to null character
        values[i] = 0.0f; // Initialize to zero
    }
    
    static_assert(sizeof...(args) % 2 == 0, "Must pass parameter/value pairs");
    parametersUsed = sizeof...(args) / 2;

    // Temporary arrays to unpack args
    char tempChars[parameterCount] = {};
    float tempVals[parameterCount] = {};

    UnpackParams(0, tempChars, tempVals, args...);

    // Copy to struct arrays
    for (uint8_t i = 0; i < parameterCount; i++) {
        if (i < parametersUsed) {
            characters[i] = tempChars[i];
            values[i] = tempVals[i];
        } else {
            characters[i] = ' ';
            values[i] = 0.0f;
        }
    }
}

template<typename TChar, typename TVal, typename... ArgsRest>
void GCodeCommand::UnpackParams(uint8_t idx, char chars[], float vals[], TChar c, TVal v, ArgsRest... rest) {
    if (idx < parameterCount) {
        chars[idx] = c;
        vals[idx] = v;
        UnpackParams(idx + 1, chars, vals, rest...);
    }
}
