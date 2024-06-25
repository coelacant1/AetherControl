#pragma once

#include "../Axis/Axis.h"
#include "../Kinematics/Cartesian.h"
#include "../Kinematics/PathPlanner.h"
#include "../Hardware/PulseControl.h"
#include "../Hardware/WS2812B.h"

#include "Implementation.h"

class PickPlaceHead : public Implementation{
private:
    AxisConstraints axisLimitZ = AxisConstraints('Z', 0.0f, 48.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitI = AxisConstraints('I', -720.0f, 720.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);

    PulseControl<2> pulseControl = PulseControl<2>();
    Cartesian<2> cartesian = Cartesian<2>();

    Axis axisZ = Axis(&pulseControl, &axisLimitZ, 6, 7, 5, 8);
    Axis axisI = Axis(&pulseControl, &axisLimitI, 3, 4, 2);

    WS2812B<20> wsLEDs = WS2812B<20>(9);

public:
    PickPlaceHead() : Implementation(GCode(&cartesian)){}

    void Initialize() override;
    void PrintInformation() override;

};
