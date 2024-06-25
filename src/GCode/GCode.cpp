#include "GCode.h"

GCode::GCode(IKinematics* kinematics) : kinematics(kinematics) {}

void GCode::AddLEDs(IWS2812B* leds){
    this->leds = leds;
}

void GCode::AddIKinematics(IKinematics* kinematics){
    this->kinematics = kinematics;
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
            switch (gc->characters[i]) {
                case IKinematics::A:
                case IKinematics::B:
                case IKinematics::E:
                case IKinematics::I:
                case IKinematics::J:
                case IKinematics::K:
                case IKinematics::U:
                case IKinematics::V:
                case IKinematics::W:
                case IKinematics::X:
                case IKinematics::Y:
                case IKinematics::Z:
                    kinematics->SetTargetPosition(gc->values[i], gc->characters[i]);
                    break;
                default:
                    break;
            }
        }
    }

    kinematics->StartMove(feedrate);
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
    kinematics->HomeAxes();
}

// Enable steppers
void GCode::M17(GCodeCommand* gc){
    if (gc->parametersUsed == 0){
        for (int i = 0; i < kinematics->GetAxisCount(); i++){
            kinematics->GetAxis(i)->Enable();
        }
    }
    else{
        for (int i = 0; i < gc->parametersUsed; i++){
            for (int j = 0; j < kinematics->GetAxisCount(); j++){
                if (kinematics->GetAxis(j)->GetAxisConstraints()->GetAxisLabel() == gc->characters[i]){
                    kinematics->GetAxis(j)->Enable();
                }
            }
        }
    }
}

// Disable steppers
void GCode::M18(GCodeCommand* gc){
    if (gc->parametersUsed == 0){
        for (int i = 0; i < kinematics->GetAxisCount(); i++){
            kinematics->GetAxis(i)->Disable();
        }
    }
    else{
        for (int i = 0; i < gc->parametersUsed; i++){
            for (int j = 0; j < kinematics->GetAxisCount(); j++){
                if (kinematics->GetAxis(j)->GetAxisConstraints()->GetAxisLabel() == gc->characters[i]){
                    kinematics->GetAxis(j)->Disable();
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
        case 4: pinMode(pin, INPUT); break;//repeat for digital
        default: SerialHandler::SendMessageValue("State", state); break;
    }

    if (state == 1) analogWrite(pin, pwmValue);
    else if (state == 0 || state == 2 || state == 3) {
        SerialHandler::SendMessageValue("Pin " + String(pin), analogRead(pin));
    }
    else if (state == 4) {
        SerialHandler::SendMessageValue("Pin " + String(pin), digitalRead(pin));
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
