#pragma once

template<size_t axisCount>
void CoreXY<axisCount>::HomeX(){
    float min = homeMaxX ? 0.0f : maxX;
    float max = homeMaxX ? maxX : 0.0f;
    float maxP5 = homeMaxX ? maxX + 5.0f : maxX - 5.0f;
    float maxM5 = homeMaxX ? maxX - 5.0f : maxX + 5.0f;

    maxHomeTime.Reset();

    // Move 5mm past extreme limit
    SetControls(min, currentY, maxP5, currentY, 1.0f / 4.0f);
    while(!ReadEndstopX() && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to trigger

    // Move 5mm below endstop distance
    SetControls(max, currentY, maxM5, currentY, 1.0f / 8.0f);
    while(!IsMoveFinished(maxM5, currentY) && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to untrigger and to move 5mm

    // Move 5mm past extreme limit
    SetControls(maxM5, currentY, maxP5, currentY, 1.0f / 12.0f);
    while(!ReadEndstopX() && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to trigger, slower

    // Move 5mm below endstop distance
    SetControls(max, currentY, maxM5, currentY, 1.0f / 4.0f);
    while(!IsMoveFinished(maxM5, currentY) && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to untrigger and to move 5mm

    currentX = maxM5;
}

template<size_t axisCount>
void CoreXY<axisCount>::HomeY(){
    float min = homeMaxY ? 0.0f : maxY;
    float max = homeMaxY ? maxY : 0.0f;
    float maxP5 = homeMaxY ? maxY + 5.0f : maxY - 5.0f;
    float maxM5 = homeMaxY ? maxY - 5.0f : maxX + 5.0f;

    maxHomeTime.Reset();

    // Move 5mm past extreme limit
    SetControls(currentX, min, currentX, maxP5, 1.0f / 4.0f);
    while(!ReadEndstopY() && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to trigger

    // Move 5mm below endstop distance
    SetControls(currentX, max, currentX, maxM5, 1.0f / 8.0f);
    while(!IsMoveFinished(currentX, maxM5) && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to untrigger and to move 5mm

    // Move 5mm past extreme limit
    SetControls(currentX, maxM5, currentX, maxP5, 1.0f / 12.0f);
    while(!ReadEndstopY() && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to trigger, slower

    // Move 5mm below endstop distance
    SetControls(currentX, max, currentX, maxM5, 1.0f / 4.0f);
    while(!IsMoveFinished(currentX, maxM5) && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to untrigger and to move 5mm
    
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
    
    this->pathPlanner.CalculateLimits(100.0f * vR);
}

template<size_t axisCount>
bool CoreXY<axisCount>::IsMoveFinished(float tX, float tY){
    return Mathematics::IsClose(axisA->GetCurrentPosition(), CalculateAPosition(tX, tY), 0.01f) && Mathematics::IsClose(axisB->GetCurrentPosition(), CalculateBPosition(tX, tY), 0.01f);
}

template<size_t axisCount>
CoreXY<axisCount>::CoreXY(uint8_t xEndstop, uint8_t yEndstop, bool homeMaxX, bool homeMaxY, float maxX, float maxY) : xEndstop(xEndstop), yEndstop(yEndstop), homeMaxX(homeMaxX), homeMaxY(homeMaxY), maxX(maxX), maxY(maxY) {
    pinMode(xEndstop, INPUT);
    pinMode(yEndstop, INPUT);
}

template<size_t axisCount>
void CoreXY<axisCount>::AddAxis(Axis* axis, IKinematics::AxisLabel axisLabel){
    if (axisLabel == 'A') axisA = axis;
    else if (axisLabel == 'B') axisB = axis;
    
    this->axes[this->currentAxes] = axis;
    this->axisLabel[this->currentAxes] = axisLabel;
    
    this->pathPlanner.AddAxis(axis);

    this->currentAxes++;
}

template<size_t axisCount>
Axis* CoreXY<axisCount>::GetAxis(uint8_t axisIndex){
    return this->axes[axisIndex];
}

template<size_t axisCount>
bool CoreXY<axisCount>::SetTargetPosition(float position, char axisLabel){
    if (axisLabel == IKinematics::X) targetX = position;
    else if (axisLabel == IKinematics::Y) targetY = position;
    else if (axisLabel == IKinematics::A || axisLabel == IKinematics::B) return false;
    else{
        for (uint8_t i = 0; i < this->currentAxes; i++){
            if (this->axisLabel[i] == axisLabel) {
                this->axisTarget[i] = position;
                return true;
            }
        }
    }
    
    return false;
}

template<size_t axisCount>
void CoreXY<axisCount>::StartMove(float feedrate){
    axisA->SetTargetPosition(CalculateAPosition(targetX, targetY));
    axisB->SetTargetPosition(CalculateBPosition(targetX, targetY));

    for (uint8_t i = 0; i < this->currentAxes; i++){// Set all at the same time
        if (this->axisLabel[i] != IKinematics::A && this->axisLabel[i] != IKinematics::B) this->axes[i]->SetTargetPosition(this->axisTarget[i]);
    }

    this->pathPlanner.CalculateLimits(feedrate);

    while (this->pathPlanner.Update()) { delay(1); }
}

template<size_t axisCount>
void CoreXY<axisCount>::HomeAxes(){
    axisA->ResetRelative();
    axisB->ResetRelative();

    axisA->Enable();
    axisB->Enable();

    HomeX();
    HomeY();

    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (this->axisLabel[i] != 'A' && this->axisLabel[i] != 'B') this->axes[i]->AutoHome();
    }
}

template<size_t axisCount>
float CoreXY<axisCount>::GetAxisPosition(char axisLabel){
    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (this->axisLabel[i] == axisLabel) {
            return this->axes[i]->GetCurrentPosition();
        }
    }

    return 0.0f;
}

template<size_t axisCount>
float CoreXY<axisCount>::GetEffectorPosition(char axisLabel){
    if (axisLabel == IKinematics::X) return currentX;
    else if (axisLabel == IKinematics::Y) return currentY;
    else if (axisLabel == IKinematics::A || axisLabel == IKinematics::B) return 0.0f;
    else{
        for (uint8_t i = 0; i < this->currentAxes; i++){
            if (this->axisLabel[i] == axisLabel) {
                return this->axes[i]->GetCurrentPosition();
            }
        }
    }

    return 0.0f;
}

template<size_t axisCount>
char CoreXY<axisCount>::GetEffectorAxisLabel(uint8_t axisIndex){
    
}
