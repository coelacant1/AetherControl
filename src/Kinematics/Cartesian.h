#pragma once

#include "Kinematics.h"

template<uint8_t axisCount>
class Cartesian : public Kinematics<axisCount>{
public:
    Cartesian();

    bool SetTargetPosition(float position, char axisLabel) override;//return if axis exists

    void StartMove(float feedrate) override;

    void StartMoveNoAccel(float feedrate) override;

    void HomeAxes() override;

    float GetAxisPosition(char axisLabel) override;

    float GetEffectorPosition(char axisLabel) override;

    char GetEffectorAxisLabel(uint8_t axisIndex) override;
    
};

#include "Cartesian.tpp"