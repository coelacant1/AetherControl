#pragma once

template<uint8_t axisCount>
void CoreXY<axisCount>::HomeX(){
    float min = homeMaxX ? 0.0f : maxX;
    float max = homeMaxX ? maxX : 0.0f;
    float maxPDist = homeMaxX ? maxX + GlobalVariables::homingDistanceBackoff : maxX - GlobalVariables::homingDistanceBackoff;
    float maxMDist = homeMaxX ? maxX - GlobalVariables::homingDistanceBackoff : maxX + GlobalVariables::homingDistanceBackoff;

    maxHomeTime.Reset();

    // Move 5mm past extreme limit
    SetControls(min, currentY, GlobalVariables::homingDistanceOffset + max, currentY, GlobalVariables::homingSpeedPrimary);
    while(!ReadEndstopX() && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to trigger

    // Move 5mm below endstop distance
    SetControls(max, currentY, maxMDist, currentY, GlobalVariables::homingSpeedSecondary);
    while(!IsMoveFinished(maxMDist, currentY) && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to untrigger and to move 5mm

    // Move 5mm past extreme limit
    SetControls(maxMDist, currentY, maxPDist, currentY, GlobalVariables::homingSpeedSecondary);
    while(!ReadEndstopX() && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to trigger, slower

    // Move 5mm below endstop distance
    SetControls(max, currentY, maxMDist, currentY, GlobalVariables::homingSpeedSecondary);
    while(!IsMoveFinished(maxMDist, currentY) && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to untrigger and to move 5mm

    currentX = maxMDist;
}

template<uint8_t axisCount>
void CoreXY<axisCount>::HomeY(){
    float min = homeMaxY ? 0.0f : maxY;
    float max = homeMaxY ? maxY : 0.0f;
    float maxPDist = homeMaxY ? maxY + GlobalVariables::homingDistanceBackoff : maxY - GlobalVariables::homingDistanceBackoff;
    float maxMDist = homeMaxY ? maxY - GlobalVariables::homingDistanceBackoff : maxX + GlobalVariables::homingDistanceBackoff;

    maxHomeTime.Reset();

    // Move 5mm past extreme limit
    SetControls(currentX, min, currentX, GlobalVariables::homingDistanceOffset + max, GlobalVariables::homingSpeedPrimary);
    while(!ReadEndstopY() && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to trigger

    // Move 5mm below endstop distance
    SetControls(currentX, max, currentX, maxMDist, GlobalVariables::homingSpeedSecondary);
    while(!IsMoveFinished(currentX, maxMDist) && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to untrigger and to move 5mm

    // Move 5mm past extreme limit
    SetControls(currentX, maxMDist, currentX, maxPDist, GlobalVariables::homingSpeedSecondary);
    while(!ReadEndstopY() && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to trigger, slower

    // Move 5mm below endstop distance
    SetControls(currentX, max, currentX, maxMDist, GlobalVariables::homingSpeedSecondary);
    while(!IsMoveFinished(currentX, maxMDist) && !maxHomeTime.IsFinished()){ this->pathPlanner.Update(); delay(1); }// Wait for endstop to untrigger and to move 5mm
    
    currentY = maxMDist;
}

template<uint8_t axisCount>
bool CoreXY<axisCount>::ReadEndstopX(){
    return digitalRead(xEndstop);
}

template<uint8_t axisCount>
bool CoreXY<axisCount>::ReadEndstopY(){
    return digitalRead(yEndstop);
}

template<uint8_t axisCount>
float CoreXY<axisCount>::CalculateAPosition(float x, float y){
    return x + y;
}

template<uint8_t axisCount>
float CoreXY<axisCount>::CalculateBPosition(float x, float y){
    return x - y;
}

template<uint8_t axisCount>
float CoreXY<axisCount>::CalculateXPosition(float a, float b){
    return (a + b) / 2.0f;
}

template<uint8_t axisCount>
float CoreXY<axisCount>::CalculateYPosition(float a, float b){
    return (a - b) / 2.0f;
}

template<uint8_t axisCount>
void CoreXY<axisCount>::SetControls(float cX, float cY, float tX, float tY, float feedrate){
    axisA->SetCurrentPosition(CalculateAPosition(cX, cY));
    axisB->SetCurrentPosition(CalculateBPosition(cX, cY));

    axisA->SetTargetPosition(CalculateAPosition(tX, tY));
    axisB->SetTargetPosition(CalculateBPosition(tX, tY));
    
    this->pathPlanner.CalculateLimits(feedrate);
}

template<uint8_t axisCount>
bool CoreXY<axisCount>::IsMoveFinished(float tX, float tY){
    return Mathematics::IsClose(axisA->GetCurrentPosition(), CalculateAPosition(tX, tY), 0.01f) && Mathematics::IsClose(axisB->GetCurrentPosition(), CalculateBPosition(tX, tY), 0.01f);
}

template<uint8_t axisCount>
CoreXY<axisCount>::CoreXY(uint8_t xEndstop, uint8_t yEndstop, bool homeMaxX, bool homeMaxY, float maxX, float maxY) : xEndstop(xEndstop), yEndstop(yEndstop), homeMaxX(homeMaxX), homeMaxY(homeMaxY), maxX(maxX), maxY(maxY) {
    pinMode(xEndstop, INPUT);
    pinMode(yEndstop, INPUT);
}

template<uint8_t axisCount>
void CoreXY<axisCount>::AddAxis(Axis* axis, IKinematics::AxisLabel axisLabel){
    if (axisLabel == 'A') axisA = axis;
    else if (axisLabel == 'B') axisB = axis;
    
    this->axes[this->currentAxes] = axis;
    this->axisLabel[this->currentAxes] = axisLabel;
    
    this->pathPlanner.AddAxis(axis);

    this->currentAxes++;
}

template<uint8_t axisCount>
Axis* CoreXY<axisCount>::GetAxis(uint8_t axisIndex){
    return this->axes[axisIndex];
}

template<uint8_t axisCount>
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

template<uint8_t axisCount>
void CoreXY<axisCount>::StartMove(float feedrate){
    axisA->SetTargetPosition(CalculateAPosition(targetX, targetY));
    axisB->SetTargetPosition(CalculateBPosition(targetX, targetY));

    for (uint8_t i = 0; i < this->currentAxes; i++){// Set all at the same time
        if (this->axisLabel[i] != IKinematics::A && this->axisLabel[i] != IKinematics::B) this->axes[i]->SetTargetPosition(this->axisTarget[i]);
    }

    this->pathPlanner.CalculateLimits(feedrate);

    while (this->pathPlanner.Update()) {}
}

template<uint8_t axisCount>
void CoreXY<axisCount>::StartMoveNoAccel(float feedrate){
    axisA->SetTargetPosition(CalculateAPosition(targetX, targetY));
    axisB->SetTargetPosition(CalculateBPosition(targetX, targetY));

    for (uint8_t i = 0; i < this->currentAxes; i++){// Set all at the same time
        if (this->axisLabel[i] != IKinematics::A && this->axisLabel[i] != IKinematics::B) this->axes[i]->SetTargetPosition(this->axisTarget[i]);
    }

    this->pathPlanner.CalculateLimitsNoAccel(feedrate);

    while (this->pathPlanner.UpdateNoAccel()) {}
}

template<uint8_t axisCount>
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

template<uint8_t axisCount>
float CoreXY<axisCount>::GetAxisPosition(char axisLabel){
    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (this->axisLabel[i] == axisLabel) {
            return this->axes[i]->GetCurrentPosition();
        }
    }

    return 0.0f;
}

template<uint8_t axisCount>
float CoreXY<axisCount>::GetEffectorPosition(char axisLabel){
    if (axisLabel == IKinematics::X) return CalculateXPosition(this->axisA->GetCurrentPosition(), this->axisB->GetCurrentPosition());
    else if (axisLabel == IKinematics::Y) return CalculateYPosition(this->axisA->GetCurrentPosition(), this->axisB->GetCurrentPosition());
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

template<uint8_t axisCount>
char CoreXY<axisCount>::GetEffectorAxisLabel(uint8_t axisIndex){
    if (axisIndex > axisCount) return IKinematics::NA;

    if (this->axes[axisIndex]->GetAxisConstraints()->GetAxisLabel() == IKinematics::A) return IKinematics::X;
    else if (this->axes[axisIndex]->GetAxisConstraints()->GetAxisLabel() == IKinematics::B) return IKinematics::Y;
    else return this->axes[axisIndex]->GetAxisConstraints()->GetAxisLabel();
}
