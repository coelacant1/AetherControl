#pragma once

#include "Kinematics.h"

// use A and B as relative axes for X and Y
template<size_t axisCount>
class CoreXY : public Kinematics<axisCount> {
private:
    const uint8_t xEndstop;
    const uint8_t yEndstop;
    const bool homeMaxX;
    const bool homeMaxY;
    const float maxX;
    const float maxY;

    Axis* axisA;
    Axis* axisB;
    
    float currentX;
    float currentY;
    float targetX;
    float targetY;

    void SetABTarget();

    void HomeX();
    void HomeY();

    bool ReadEndstopX();
    bool ReadEndstopY();

    float CalculateAPosition(float x, float y);
    float CalculateBPosition(float x, float y);

    void SetControls(float cX, float cY, float tX, float tY, float vR);
    bool IsMoveFinished();

public:
    CoreXY(IPathPlanner* pathPlanner, uint8_t xEndstop, uint8_t yEndstop, bool homeMaxX, bool homeMaxY, float maxX, float maxY);

    void AddAxis(Axis* axis, IKinematics::AxisLabel axisLabel) override;
    
    Axis* GetAxis(char axisLabel) override;

    bool SetTargetPosition(float position, char axisLabel) override;//return if axis exists

    void StartMove(float feedrate) override;

    void HomeAxes() override;

};

template<size_t axisCount>
void CoreXY<axisCount>::SetABTarget(){
    axisA->SetTargetPosition(CalculateAPosition(targetX, targetY));
    axisB->SetTargetPosition(CalculateBPosition(targetX, targetY));
}

template<size_t axisCount>
void CoreXY<axisCount>::HomeX(){
    float min = homeMaxX ? 0.0f : maxX;
    float max = homeMaxX ? maxX : 0.0f;
    float maxP5 = homeMaxX ? maxX + 5.0f : maxX - 5.0f;
    float maxM5 = homeMaxX ? maxX - 5.0f : maxX + 5.0f;

    // Move 5mm past extreme limit
    SetControls(min, currentY, maxP5, currentY, 1.0f / 25.0f);
    while(!ReadEndstopX()){ this->pathPlanner->Update(); delay(5); }// Wait for endstop to trigger

    // Move 5mm below endstop distance
    SetControls(max, currentY, maxM5, currentY, 1.0f / 25.0f);
    while(ReadEndstopX() || !IsMoveFinished()){ this->pathPlanner->Update(); delay(5); }// Wait for endstop to untrigger and to move 5mm

    // Move 5mm past extreme limit
    SetControls(maxM5, currentY, maxP5, currentY, 1.0f / 50.0f);
    while(!ReadEndstopX()){ this->pathPlanner->Update(); delay(5); }// Wait for endstop to trigger, slower

    // Move 5mm below endstop distance
    SetControls(max, currentY, maxM5, currentY, 1.0f / 25.0f);
    while(ReadEndstopX() || !IsMoveFinished()){ this->pathPlanner->Update(); delay(5); }// Wait for endstop to untrigger and to move 5mm

    currentX = maxM5;
}

template<size_t axisCount>
void CoreXY<axisCount>::HomeY(){
    float min = homeMaxY ? 0.0f : maxY;
    float max = homeMaxY ? maxY : 0.0f;
    float maxP5 = homeMaxY ? maxY + 5.0f : maxY - 5.0f;
    float maxM5 = homeMaxY ? maxY - 5.0f : maxX + 5.0f;

    // Move 5mm past extreme limit
    SetControls(currentX, min, currentX, maxP5, 1.0f / 25.0f);
    while(!ReadEndstopY()){ this->pathPlanner->Update(); delay(5); }// Wait for endstop to trigger

    // Move 5mm below endstop distance
    SetControls(currentX, max, currentX, maxM5, 1.0f / 25.0f);
    while(ReadEndstopY() || !IsMoveFinished()){ this->pathPlanner->Update(); delay(5); }// Wait for endstop to untrigger and to move 5mm

    // Move 5mm past extreme limit
    SetControls(currentX, maxM5, currentX, maxP5, 1.0f / 50.0f);
    while(!ReadEndstopY()){ this->pathPlanner->Update(); delay(5); }// Wait for endstop to trigger, slower

    // Move 5mm below endstop distance
    SetControls(currentX, max, currentX, maxM5, 1.0f / 25.0f);
    while(ReadEndstopY() || !IsMoveFinished()){ this->pathPlanner->Update(); delay(5); }// Wait for endstop to untrigger and to move 5mm
    
    currentY = maxM5;
}

template<size_t axisCount>
bool CoreXY<axisCount>::ReadEndstopX(){
    return digitalRead(xEndstop);
}

template<size_t axisCount>
bool CoreXY<axisCount>::ReadEndstopY(){
    return digitalRead(yEndstop);
}

template<size_t axisCount>
float CoreXY<axisCount>::CalculateAPosition(float x, float y){
    return x + y;
}

template<size_t axisCount>
float CoreXY<axisCount>::CalculateBPosition(float x, float y){
    return x - y;
}

template<size_t axisCount>
void CoreXY<axisCount>::SetControls(float cX, float cY, float tX, float tY, float vR){
    axisA->SetCurrentPosition(CalculateAPosition(cX, cY));
    axisB->SetCurrentPosition(CalculateBPosition(cX, cY));

    axisA->SetTargetPosition(CalculateAPosition(tX, tY));
    axisB->SetTargetPosition(CalculateBPosition(tX, tY));

    axisA->SetTargetVelocity(axisA->GetAxisConstraints()->GetMaxVelocity() * vR);
    axisB->SetTargetVelocity(axisB->GetAxisConstraints()->GetMaxVelocity() * vR);
    
    this->pathPlanner->CalculateLimits(25.0f);
}

template<size_t axisCount>
bool CoreXY<axisCount>::IsMoveFinished(){
    return Mathematics::IsClose(axisA->GetCurrentPosition(), axisA->GetTargetPosition(), 0.01f) || Mathematics::IsClose(axisB->GetCurrentPosition(), axisB->GetTargetPosition(), 0.01f);
}

template<size_t axisCount>
CoreXY<axisCount>::CoreXY(IPathPlanner* pathPlanner, uint8_t xEndstop, uint8_t yEndstop, bool homeMaxX, bool homeMaxY, float maxX, float maxY) : Kinematics<axisCount>(pathPlanner), xEndstop(xEndstop), yEndstop(yEndstop), homeMaxX(homeMaxX), homeMaxY(homeMaxY), maxX(maxX), maxY(maxY) {
    pinMode(xEndstop, INPUT_PULLUP);
    pinMode(yEndstop, INPUT_PULLUP);
}

template<size_t axisCount>
void CoreXY<axisCount>::AddAxis(Axis* axis, IKinematics::AxisLabel axisLabel){
    if (axisLabel == 'A') {
        axisA = axis;
        this->pathPlanner->AddAxis(axisA);
    }
    else if (axisLabel == 'B') {
        axisB = axis;
        this->pathPlanner->AddAxis(axisB);
    }
    else if (this->currentAxes < axisCount){
        this->axes[this->currentAxes] = axis;
        this->axisLabel[this->currentAxes] = axisLabel;

        this->currentAxes++;
    }
}

template<size_t axisCount>
Axis* CoreXY<axisCount>::GetAxis(char axisLabel){
    if (axisLabel == 'A') return axisA;
    else if (axisLabel == 'B') return axisB;
    
    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (this->axisLabel[i] == axisLabel) return this->axes[i];
    }

    return nullptr;
}

template<size_t axisCount>
bool CoreXY<axisCount>::SetTargetPosition(float position, char axisLabel){
    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (axisLabel == 'X') {
            targetX = position;
            return true;
        }
        else if (axisLabel == 'Y') {
            targetY = position;
            return true;
        }
        else if (this->axisLabel[i] == axisLabel) {
            this->axisTarget[i] = position;
            return true;
        }
    }

    return false;
}

template<size_t axisCount>
void CoreXY<axisCount>::StartMove(float feedrate){
    SetABTarget();

    for (uint8_t i = 0; i < this->currentAxes; i++){// Set all at the same time
        this->axes[i]->SetTargetPosition(this->axisTarget[i]);
    }
    
    this->pathPlanner->CalculateLimits(feedrate);

    while (this->pathPlanner->Update()) delay(10);
}

template<size_t axisCount>
void CoreXY<axisCount>::HomeAxes(){
    axisA->ResetRelative();
    axisB->ResetRelative();

    HomeX();
    HomeY();

    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (this->axisLabel[i] != 'A' && this->axisLabel[i] != 'B') this->axes[i]->AutoHome();
    }
}
