#pragma once

template<size_t axisCount>
Cartesian<axisCount>::Cartesian() {}

template<size_t axisCount>
bool Cartesian<axisCount>::SetTargetPosition(float position, char axisLabel){
    for (uint8_t i = 0; i < this->currentAxes; i++){
        if (this->axisLabel[i] == axisLabel) {
            this->axisTarget[i] = position;
            return true;
        }
    }
    
    return false;
}

template<size_t axisCount>
void Cartesian<axisCount>::StartMove(float feedrate){
    for (uint8_t i = 0; i < this->currentAxes; i++){// Set all at the same time
        this->axes[i]->SetTargetPosition(this->axisTarget[i]);
    }
    
    this->pathPlanner.CalculateLimits(feedrate);

    while (this->pathPlanner.Update()) delay(10);
}

template<size_t axisCount>
void Cartesian<axisCount>::HomeAxes(){
    for (uint8_t i = 0; i < this->currentAxes; i++){
        if(this->axes[i]->IsRelative()) this->axes[i]->ResetRelative();
        else this->axes[i]->AutoHome();
    }
}
