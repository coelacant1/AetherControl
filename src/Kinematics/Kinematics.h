#pragma once

#include "IKinematics.h"
#include "PathPlanner.h"

template<uint8_t axisCount>
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
    virtual float GetAxisPosition(char axisLabel) = 0;
    virtual float GetEffectorPosition(char axisLabel) = 0;
    virtual char GetEffectorAxisLabel(uint8_t axisIndex) = 0;

};

#include "Kinematics.tpp"
