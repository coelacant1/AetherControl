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

#include "Cartesian.tpp"