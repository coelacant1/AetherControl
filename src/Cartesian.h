#pragma once

#include "Kinematics.h"

template<size_t axisCount>
class Cartesian : public Kinematics<axisCount>{
public:
    Cartesian();

    bool SetTargetPosition(float position, char axisLabel) override;//return if axis exists

    void StartMove(float feedrate) override;

    void HomeAxes() override;
};

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
