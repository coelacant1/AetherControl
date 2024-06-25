//handles list of gcode command objects
//iterates through list
#pragma once

#include "../lib/ProtoTracer/Utils/Math/Mathematics.h"

#include "IPathPlanner.h"

template<size_t axisCount>
class PathPlanner : public IPathPlanner {
private:
    Axis* axes[axisCount];
    float startPosition[axisCount];
    float endPosition[axisCount];
    uint8_t currentAxes = 0;
    elapsedMicros sinceUpdate;

    float velocity = 0.0f;
    float targetVelocity = 0.0f;
    float acceleration = 0.0f;
    float ratio = 0.0f;

    bool newCommand = true;

public:
    PathPlanner();

    void AddAxis(Axis* axis) override;
    Axis* GetAxis(int axisIndex) override;
    uint8_t GetAxisCount() override;
    void CalculateLimits(float feedrate) override;
    bool Update() override;

};

#include "PathPlanner.tpp"
