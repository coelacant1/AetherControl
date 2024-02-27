#pragma once

#include "Axis.h"
#include "GCodeCommand.h"
#include "SerialHandler.h"
#include "IWS2812B.h"
#include "IPathPlanner.h"

// Class for implementation of function calls for gcode commands
class GCode {
private:
    IWS2812B* leds;
    IPathPlanner* pathPlanner;

public:
    GCode(IPathPlanner* pathPlanner);

    void AddLEDs(IWS2812B* leds);
    void AddPathPlanner(IPathPlanner* pathPlanner);

    void ExecuteGCode(GCodeCommand* gc);

    void G0(GCodeCommand* gc);// Rapid move
    void G1(GCodeCommand* gc);// Linear move
    void G4(GCodeCommand* gc);// Dwell
    void G28(GCodeCommand* gc);// Home all non-relative axes

    void M17(GCodeCommand* gc);// Enable steppers
    void M18(GCodeCommand* gc);// Disable steppers
    void M42(GCodeCommand* gc);// Manual pin control
    void M150(GCodeCommand* gc);// Set LED color
    
};

GCode::GCode(IPathPlanner* pathPlanner) : pathPlanner(pathPlanner) {}

void GCode::AddLEDs(IWS2812B* leds){
    this->leds = leds;
}

void GCode::AddPathPlanner(IPathPlanner* pathPlanner){
    this->pathPlanner = pathPlanner;
}

void GCode::ExecuteGCode(GCodeCommand* gc){
    if (gc->commandType == 'G'){
        switch (gc->commandNumber){
            case   0: G0(gc);  break;
            case   1: G1(gc);  break;
            case   4: G4(gc);  break;
            case  28: G28(gc); break;
            default: SerialHandler::SendNotImplemented(); break;
        }
    }
    else if (gc->commandType == 'M'){
        switch (gc->commandNumber){
            case  17: M17(gc);  break;
            case  18: M18(gc);  break;
            case  42: M42(gc);  break;
            case 150: M150(gc); break;
            default: SerialHandler::SendNotImplemented(); break;
        }
    }
    else{
        SerialHandler::SendNotImplemented();
    }
}

// Rapid move
void GCode::G0(GCodeCommand* gc){
    float feedrate = 10.0f;

    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'F'){
            feedrate = gc->values[i];
            continue;
        }
        else{
            for (int j = 0; j < pathPlanner->GetAxisCount(); j++){
                if (pathPlanner->GetAxis(j)->GetAxisConstraints()->GetAxisLabel() == gc->characters[i]){
                    pathPlanner->GetAxis(j)->SetTargetPosition(gc->values[i]);
                    break;
                }
            }
        }
    }

    pathPlanner->CalculateLimits(feedrate);

    while (pathPlanner->Update()) delay(10);
}

// Linear move
void GCode::G1(GCodeCommand* gc){
    G0(gc);
}

// Dwell
void GCode::G4(GCodeCommand* gc){
    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'P'){// Millis
            delay(gc->values[i]);
        }
        else if (gc->characters[i] == 'S'){// Seconds
            delay(gc->values[i] * 1000);
        }
    }
}

// Home all non-relative axes
void GCode::G28(GCodeCommand* gc){
    if (gc->parametersUsed == 0){
        for (int i = 0; i < pathPlanner->GetAxisCount(); i++){
            if (pathPlanner->GetAxis(i)->IsRelative()) pathPlanner->GetAxis(i)->ResetRelative();
            else pathPlanner->GetAxis(i)->AutoHome();
        }
    }
    else{
        for (int i = 0; i < gc->parametersUsed; i++){
            for (int j = 0; j < pathPlanner->GetAxisCount(); j++){
                if (pathPlanner->GetAxis(j)->GetAxisConstraints()->GetAxisLabel() == gc->characters[i]){
                    if (pathPlanner->GetAxis(j)->IsRelative()) pathPlanner->GetAxis(j)->ResetRelative();
                    else pathPlanner->GetAxis(j)->AutoHome();
                }
            }
        }
    }
}

// Enable steppers
void GCode::M17(GCodeCommand* gc){
    if (gc->parametersUsed == 0){
        for (int i = 0; i < pathPlanner->GetAxisCount(); i++){
            pathPlanner->GetAxis(i)->Enable();
        }
    }
    else{
        for (int i = 0; i < gc->parametersUsed; i++){
            for (int j = 0; j < pathPlanner->GetAxisCount(); j++){
                if (pathPlanner->GetAxis(j)->GetAxisConstraints()->GetAxisLabel() == gc->characters[i]){
                    pathPlanner->GetAxis(j)->Enable();
                }
            }
        }
    }
}

// Disable steppers
void GCode::M18(GCodeCommand* gc){
    if (gc->parametersUsed == 0){
        for (int i = 0; i < pathPlanner->GetAxisCount(); i++){
            pathPlanner->GetAxis(i)->Disable();
        }
    }
    else{
        for (int i = 0; i < gc->parametersUsed; i++){
            for (int j = 0; j < pathPlanner->GetAxisCount(); j++){
                if (pathPlanner->GetAxis(j)->GetAxisConstraints()->GetAxisLabel() == gc->characters[i]){
                    pathPlanner->GetAxis(j)->Disable();
                }
            }
        }
    }
}

// Manual pin control
void GCode::M42(GCodeCommand* gc){
    uint8_t pin = 255;
    uint8_t pwmValue = 0;
    uint8_t state = 0;

    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'P') pin = gc->values[i];
        if (gc->characters[i] == 'S') pwmValue = gc->values[i];
        if (gc->characters[i] == 'T') state = gc->values[i];
    }

    switch (state){
        case 0: pinMode(pin, INPUT); break;
        case 1: pinMode(pin, OUTPUT); break;
        case 2: pinMode(pin, INPUT_PULLUP); break;
        case 3: pinMode(pin, INPUT_PULLDOWN); break;
        default: SerialHandler::SendMessageValue("State", state); break;
    }

    if (state == 1) analogWrite(pin, pwmValue);
    else if (state == 0 || state == 2 || state == 3) {
        SerialHandler::SendMessageValue("Pin " + String(pin), analogRead(pin));
    }
}

// Set LED color
void GCode::M150(GCodeCommand* gc){
    uint8_t R = 0, G = 0, B = 0;

    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'R')      R = gc->values[i];
        else if (gc->characters[i] == 'G') G = gc->values[i];
        else if (gc->characters[i] == 'B') B = gc->values[i];
    }
    
    leds->SetColor(R, G, B);

    leds->Update();
}
