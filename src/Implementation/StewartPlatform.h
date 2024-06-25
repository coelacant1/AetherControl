#pragma once

#include "../Axis/Axis.h"
#include "../Kinematics/Cartesian.h"
#include "../Kinematics/PathPlanner.h"
#include "../Hardware/PulseControl.h"

#include "Implementation.h"

class StewartPlatform : public Implementation{
private:
    AxisConstraints axisLimitX = AxisConstraints('X', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitY = AxisConstraints('Y', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitZ = AxisConstraints('Z', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitU = AxisConstraints('U', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitV = AxisConstraints('V', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);
    AxisConstraints axisLimitW = AxisConstraints('W', 0.0f, 1000.0f, 0.01f, 1000.0f, 10000.0f, 1200.0f, 80.0f);

    PulseControl<6> pulseControl = PulseControl<6>();
    Cartesian<6> cartesian = Cartesian<6>();

    Axis axisX = Axis(&pulseControl, &axisLimitX, 6, 7, 5);
    Axis axisY = Axis(&pulseControl, &axisLimitY, 6, 7, 5);
    Axis axisZ = Axis(&pulseControl, &axisLimitZ, 6, 7, 5);
    Axis axisU = Axis(&pulseControl, &axisLimitU, 6, 7, 5);
    Axis axisV = Axis(&pulseControl, &axisLimitV, 6, 7, 5);
    Axis axisW = Axis(&pulseControl, &axisLimitW, 6, 7, 5);

public:
    StewartPlatform():Implementation(GCode(&cartesian)){}

    void Initialize() override;
    void PrintInformation() override;

};
