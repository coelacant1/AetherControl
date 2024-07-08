#pragma once

template <typename T>
void SerialHandler::SendMessageValue(String message, T value){
    if(serialType) {
        serialHS->print(message); serialHS->print(":");
        serialHS->println(value);
    }
    else {
        serialUSC->print(message); serialUSC->print(":");
        serialUSC->println(value);
    }
}

template <typename T>
void SerialHandler::SendMessageValueSpace(String message, T value){
    if(serialType) {
        serialHS->print(message); serialHS->print(":"); serialHS->print(value); 
        serialHS->print(" ");
    }
    else {
        serialUSC->print(message); serialUSC->print(":"); serialUSC->print(value); 
        serialUSC->print(" ");
    }
}

template <typename T>
void SerialHandler::SendMessageValues(String message, uint8_t count, T* value){
    if(serialType) {
        serialHS->print(message); serialHS->print(":");

        for (uint8_t i = 0; i < count; i++){
            serialHS->print(value[i]);
            
            if (i != count) serialHS->print(",");
        }
        
        serialHS->println();
    }
    else {
        serialUSC->print(message); serialUSC->print(":");

        for (uint8_t i = 0; i < count; i++){
            serialUSC->print(value[i]);
            
            if (i != count) serialUSC->print(",");
        }

        serialUSC->println();
    }
}
