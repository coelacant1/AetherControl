#pragma once

#include "../Axis/Axis.h"
#include "../Kinematics/Cartesian.h"
#include "../Kinematics/PathPlanner.h"
#include "../Hardware/PulseControl.h"

#include "Implementation.h"

class StewartPlatform : public Implementation{
private:
    AxisConstraints axisLimitX = AxisConstraints('X', 0.0f, 425.0f, 10.0f, 1000.0f, 10000.0f, 50.0f, 75.0f);
    AxisConstraints axisLimitY = AxisConstraints('Y', 0.0f, 425.0f, 10.0f, 1000.0f, 10000.0f, 50.0f, 75.0f);
    AxisConstraints axisLimitZ = AxisConstraints('Z', 0.0f, 425.0f, 10.0f, 1000.0f, 10000.0f, 50.0f, 75.0f);
    AxisConstraints axisLimitU = AxisConstraints('U', 0.0f, 425.0f, 10.0f, 1000.0f, 10000.0f, 50.0f, 75.0f);
    AxisConstraints axisLimitV = AxisConstraints('V', 0.0f, 425.0f, 10.0f, 1000.0f, 10000.0f, 50.0f, 75.0f);
    AxisConstraints axisLimitW = AxisConstraints('W', 0.0f, 425.0f, 10.0f, 1000.0f, 10000.0f, 50.0f, 75.0f);

    PulseControl<6> pulseControl = PulseControl<6>();
    Cartesian<6> cartesian = Cartesian<6>();

    Axis axisX = Axis(&pulseControl, &axisLimitX, 1, 2, 0);
    Axis axisY = Axis(&pulseControl, &axisLimitY, 4, 5, 3);
    Axis axisZ = Axis(&pulseControl, &axisLimitZ, 7, 8, 6);
    Axis axisU = Axis(&pulseControl, &axisLimitU, 10, 11, 9);
    Axis axisV = Axis(&pulseControl, &axisLimitV, 18, 17, 12);
    Axis axisW = Axis(&pulseControl, &axisLimitW, 15, 14, 16);

public:
    StewartPlatform():Implementation(GCode(&cartesian)){}

    void Initialize() override;
    void PrintInformation() override;

};
