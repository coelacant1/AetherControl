#pragma once

#include "../Axis/Axis.h"
#include "../Kinematics/CoreXY.h"
#include "../Kinematics/PathPlanner.h"
#include "../Hardware/PulseControl.h"
#include "../Hardware/WS2812B.h"
#include "Implementation.h"

class PickPlace : public Implementation{
private:
    AxisConstraints axisLimitA = AxisConstraints('A', 0.0f, 1000.0f, 0.01f, 1000.0f, 50000.0f, 20000.0f, 160.0f);
    AxisConstraints axisLimitB = AxisConstraints('B', 0.0f, 1000.0f, 0.01f, 1000.0f, 50000.0f, 20000.0f, 160.0f);

    PulseControl<2> pulseControl = PulseControl<2>();
    CoreXY<2> coreXY = CoreXY<2>(18, 16, true, true, 270, 415);

    Axis axisA = Axis(&pulseControl, &axisLimitA, 32, 31, 30);
    Axis axisB = Axis(&pulseControl, &axisLimitB, 8, 7, 6);

    WS2812B<20> wsLEDs = WS2812B<20>(19);

public:
    PickPlace() : Implementation(GCode(&coreXY)){}

    void Initialize() override;
    void PrintInformation() override;

};
