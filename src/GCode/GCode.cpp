#include "GCode.h"

GCode::GCode(IKinematics* kinematics) : kinematics(kinematics) {}

void GCode::AddLEDs(IWS2812B* leds){
    this->leds = leds;
}

void GCode::AddIKinematics(IKinematics* kinematics){
    this->kinematics = kinematics;
}

void GCode::ExecuteGCode(const GCodeCommand* gc){
    if (gc->commandType == 'G'){
        switch (gc->commandNumber){
            case   0: G0(gc);  break;
            case   1: G1(gc);  break;
            case   4: G4(gc);  break;
            case   6: G6(gc);  break;
            case  21: G21(gc); break;
            case  28: G28(gc); break;
            case  90: G90(gc); break;
            case  91: G91(gc); break;
            case  92: G92(gc); break;
            default: SerialHandler::SendNotImplemented(); break;
        }
    }
    else if (gc->commandType == 'M'){
        switch (gc->commandNumber){
            case  17: M17(gc);  break;
            case  18: M18(gc);  break;
            case  42: M42(gc);  break;
            case 114: M114(gc); break;
            case 115: M115(gc); break;
            case 150: M150(gc); break;
            case 204: M204(gc); break;
            case 400: M400(gc); break;
            default: SerialHandler::SendNotImplemented(); break;
        }
    }
    else{
        SerialHandler::SendNotImplemented();
    }
}

// Rapid move
void GCode::G0(const GCodeCommand* gc){
    if (gc->parametersUsed == 0) return;

    float feedrate = 10.0f;

    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'F'){
            feedrate = gc->values[i] / 60.0f;//input at mm/min convert to mm/s
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
                case IKinematics::Z: {
                    float target = GlobalVariables::relativeMode ? gc->values[i] + kinematics->GetEffectorPosition(gc->characters[i]) : gc->values[i];

                    kinematics->SetTargetPosition(target, gc->characters[i]);
                    break;
                }
                default:
                    break;
            }
        }
    }

    kinematics->StartMove(feedrate);
}

// Linear move
void GCode::G1(const GCodeCommand* gc){
    G0(gc);
}

// Dwell
void GCode::G4(const GCodeCommand* gc){
    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'P'){// Millis
            delay(gc->values[i]);
        }
        else if (gc->characters[i] == 'S'){// Seconds
            delay(gc->values[i] * 1000);
        }
    }
}

// Direct Stepper Move, no acceleration
void GCode::G6(const GCodeCommand* gc){
    if (gc->parametersUsed == 0) return;

    float feedrate = 10.0f;

    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'F'){
            feedrate = gc->values[i] / 60.0f;//input at mm/min convert to mm/s
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
                case IKinematics::Z: {
                    float target = GlobalVariables::relativeMode ? gc->values[i] + kinematics->GetEffectorPosition(gc->characters[i]) : gc->values[i];

                    kinematics->SetTargetPosition(target, gc->characters[i]);
                    break;
                }
                default:
                    break;
            }
        }
    }

    kinematics->StartMoveNoAccel(feedrate);
}

// Set Millimeter mode
void GCode::G21(const GCodeCommand* gc){}

// Home all non-relative axes
void GCode::G28(const GCodeCommand* gc){
    kinematics->HomeAxes();
    
    // Initialize default target position to current position
    for (uint8_t i = 0; i < kinematics->GetAxisCount(); i++){
        char axisLabel = kinematics->GetEffectorAxisLabel(i);

        kinematics->SetTargetPosition(kinematics->GetEffectorPosition(axisLabel), axisLabel);
    }
}

// Absolute positioning mode
void GCode::G90(const GCodeCommand* gc){
    GlobalVariables::relativeMode = false;
}

// Relative positioning mode
void GCode::G91(const GCodeCommand* gc){
    GlobalVariables::relativeMode = true;
}

// Set global offsets
void GCode::G92(const GCodeCommand* gc){}

// Enable steppers
void GCode::M17(const GCodeCommand* gc){
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
void GCode::M18(const GCodeCommand* gc){
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
void GCode::M42(const GCodeCommand* gc){
    uint8_t pin = 255;
    uint8_t pwmValue = 0;
    uint8_t state = 0;

    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'P') pin = gc->values[i];
        if (gc->characters[i] == 'S') pwmValue = gc->values[i];
        if (gc->characters[i] == 'T') state = gc->values[i];
    }

    switch (state){
        case 0: case 1: pinMode(pin, OUTPUT); break;
        case 2: case 3: pinMode(pin, INPUT); break;
        case 4: pinMode(pin, INPUT_PULLUP); break;
        case 5: pinMode(pin, INPUT_PULLDOWN); break;
        default: SerialHandler::SendMessageValue("State", state); break;
    }

    delay(10);

    if (state == 0) digitalWrite(pin, pwmValue);
    else if (state == 1) analogWrite(pin, pwmValue);
    else if (state == 3) {
        SerialHandler::SendMessageValue("Pin " + String(pin), digitalRead(pin));
    }
    else if (state == 4 || state == 5 || state == 6) {
        SerialHandler::SendMessageValue("Pin " + String(pin), analogRead(pin));
    }
}

// Get Position
void GCode::M114(const GCodeCommand* gc){
    for (int i = 0; i < kinematics->GetAxisCount(); i++){
        char axisLabel = kinematics->GetEffectorAxisLabel(i);

        SerialHandler::SendMessageValueSpace(axisLabel, kinematics->GetEffectorPosition(axisLabel));
    }

    SerialHandler::SendMessage("");
}

// Detect Firmware
void GCode::M115(const GCodeCommand* gc){
    SerialHandler::SendMessageNLN("FIRMWARE_NAME:");
    SerialHandler::SendMessageSpace(GlobalVariables::firmwareName);
    SerialHandler::SendMessageSpace(GlobalVariables::firmwareVersion);
    SerialHandler::SendMessageNLN("SOURCE_CODE_URL:");
    SerialHandler::SendMessage(GlobalVariables::firmwareSourceURL);
}

// Set LED color
void GCode::M150(const GCodeCommand* gc){
    uint8_t R = 0, G = 0, B = 0;

    for (int i = 0; i < gc->parametersUsed; i++){
        if (gc->characters[i] == 'R')      R = gc->values[i];
        else if (gc->characters[i] == 'G') G = gc->values[i];
        else if (gc->characters[i] == 'B') B = gc->values[i];
    }
    
    leds->SetColor(R, G, B);

    leds->Update();
}

// Set Acceleration
void GCode::M204(const GCodeCommand* gc){    
    for (int i = 0; i < gc->parametersUsed; i++){
        for (int j = 0; j < kinematics->GetAxisCount(); j++) {
            if (gc->characters[i] == 'T' || gc->characters[i] == 'P' || gc->characters[i] == 'S') kinematics->GetAxis(j)->GetAxisConstraints()->SetAcceleration(gc->values[i]);
        }
    }
}

// Wait for move completion, nothing required, synchronous control only
void GCode::M400(const GCodeCommand* gc){}