#pragma once

template<uint8_t axisCount>
Cartesian<axisCount>::Cartesian() {}

template<uint8_t axisCount>
bool Cartesian<axisCount>::SetTargetPosition(float position, char axisLabel){
    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (this->axisLabel[i] == axisLabel) {
            this->axisTarget[i] = position;
            return true;
        }
    }
    
    return false;
}

template<uint8_t axisCount>
void Cartesian<axisCount>::StartMove(float feedrate){
    for (uint8_t i = 0; i < this->currentAxes; i++){// Set all at the same time
        this->axes[i]->SetTargetPosition(this->axisTarget[i]);
    }
    
    this->pathPlanner.CalculateLimits(feedrate);

    while (this->pathPlanner.Update()) delay(10);
}

template<uint8_t axisCount>
void Cartesian<axisCount>::HomeAxes(){
    for (uint8_t i = 0; i < this->currentAxes; i++){
        this->axes[i]->Enable();

        if(this->axes[i]->IsRelative()) this->axes[i]->ResetRelative();
        else this->axes[i]->AutoHome();
    }
}

template<uint8_t axisCount>
float Cartesian<axisCount>::GetAxisPosition(char axisLabel){
    return GetEffectorPosition(axisLabel);
}

template<uint8_t axisCount>
float Cartesian<axisCount>::GetEffectorPosition(char axisLabel){
    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (this->axisLabel[i] == axisLabel) {
            return this->axes[i]->GetCurrentPosition();
        }
    }

    return 0.0f;
}

template<uint8_t axisCount>
char Cartesian<axisCount>::GetEffectorAxisLabel(uint8_t axisIndex){
    if (axisIndex > axisCount) return IKinematics::NA;
    
    return this->axes[axisIndex]->GetAxisConstraints()->GetAxisLabel();
}