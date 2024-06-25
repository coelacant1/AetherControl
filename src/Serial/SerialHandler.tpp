#pragma once

template <typename T>
void SerialHandler::SendMessageValue(String message, T value){
    if(serialType) {
        serialHS->print(message); serialHS->print(": ");
        serialHS->println(value);
    }
    else {
        serialUSC->print(message); serialUSC->print(": ");
        serialUSC->println(value);
    }
}