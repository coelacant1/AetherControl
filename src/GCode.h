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
    void G28(GCodeCommand* gc);// Home all non-relative axes

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
            case  28: G28(gc); break;
            default: SerialHandler::SendNotImplemented(); break;
        }
    }
    else if (gc->commandType == 'M'){
        switch (gc->commandNumber){
            case 150: M150(gc); break;
            default: SerialHandler::SendNotImplemented(); break;
        }
    }
    else{
        SerialHandler::SendNotImplemented();

        Serial.print("Error, CMD:\t");
        Serial.print(gc->commandType); Serial.print('\t');
        Serial.print(gc->commandNumber); Serial.print('\t');
        Serial.print(gc->characters[0]); Serial.print('\t');
        Serial.print(gc->values[0]);
        Serial.println();
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

// Home all non-relative axes
void GCode::G28(GCodeCommand* gc){
    for (int i = 0; i < pathPlanner->GetAxisCount(); i++){
        if (pathPlanner->GetAxis(i)->IsRelative()) pathPlanner->GetAxis(i)->ResetRelative();
        else pathPlanner->GetAxis(i)->AutoHome();
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
