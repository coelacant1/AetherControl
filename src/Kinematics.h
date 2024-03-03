#pragma once

#include "IKinematics.h"
#include "IPathPlanner.h"

template<size_t axisCount>
class Kinematics : public IKinematics{
protected:
    IPathPlanner* pathPlanner;
    
    Axis* axes[axisCount];
    AxisLabel axisLabel[axisCount];
    float axisTarget[axisCount];
    uint8_t currentAxes = 0;

public:
    Kinematics(IPathPlanner* pathPlanner) : pathPlanner(pathPlanner){}

    void AddAxis(Axis* axis, AxisLabel axisLabel) override;
    Axis* GetAxis(char axisLabel) override;
    uint8_t GetAxisCount() override;

    virtual bool SetTargetPosition(float position, char axisLabel) = 0;//return if axis exists
    virtual void StartMove(float feedrate) = 0;
    virtual void HomeAxes() = 0;

};

template<size_t axisCount>
void Kinematics<axisCount>::AddAxis(Axis* axis, AxisLabel axisLabel){
    if (currentAxes < axisCount){
        this->axes[currentAxes] = axis;
        this->axisLabel[currentAxes] = axisLabel;

        currentAxes++;
    }
}

template<size_t axisCount>
Axis* Kinematics<axisCount>::GetAxis(char axisLabel){
    for (uint8_t i = 0; i < currentAxes; i++){
        if (this->axisLabel[i] == axisLabel) return axes[i];
    }

    return nullptr;
}

template<size_t axisCount>
uint8_t Kinematics<axisCount>::GetAxisCount(){
    return currentAxes;
}
