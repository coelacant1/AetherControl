#pragma once

#include "IKinematics.h"
#include "PathPlanner.h"

template<size_t axisCount>
class Kinematics : public IKinematics{
protected:
    PathPlanner<axisCount> pathPlanner;
    
    Axis* axes[axisCount];
    AxisLabel axisLabel[axisCount];
    float axisTarget[axisCount];
    uint8_t currentAxes = 0;

public:
    Kinematics(){}

    void AddAxis(Axis* axis, AxisLabel axisLabel) override;
    Axis* GetAxis(uint8_t axisIndex) override;
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
        
        pathPlanner.AddAxis(axis);

        currentAxes++;
    }
}

template<size_t axisCount>
Axis* Kinematics<axisCount>::GetAxis(uint8_t axisIndex){
    return axes[axisIndex];
}

template<size_t axisCount>
uint8_t Kinematics<axisCount>::GetAxisCount(){
    return currentAxes;
}
