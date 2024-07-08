#pragma once

#include "Kinematics.h"

#include "../lib/ProtoTracer/Utils/Math/Mathematics.h"
#include "../lib/ProtoTracer/Utils/Time/Wait.h"

// use A and B as relative axes for X and Y
template<uint8_t axisCount>
class CoreXY : public Kinematics<axisCount> {
private:
    const uint8_t xEndstop;
    const uint8_t yEndstop;
    const bool homeMaxX;
    const bool homeMaxY;
    const float maxX;
    const float maxY;

    Axis* axisA;
    Axis* axisB;
    Wait maxHomeTime = Wait(30000);
    
    float currentX = 0.0f;
    float currentY = 0.0f;
    float targetX = 0.0f;
    float targetY = 0.0f;

    void HomeX();
    void HomeY();

    bool ReadEndstopX();
    bool ReadEndstopY();

    float CalculateAPosition(float x, float y);
    float CalculateBPosition(float x, float y);
    float CalculateXPosition(float a, float b);
    float CalculateYPosition(float a, float b);

    void SetControls(float cX, float cY, float tX, float tY, float vR);
    bool IsMoveFinished(float tX, float tY);

public:
    CoreXY(uint8_t xEndstop, uint8_t yEndstop, bool homeMaxX, bool homeMaxY, float maxX, float maxY);

    void AddAxis(Axis* axis, IKinematics::AxisLabel axisLabel) override;
    
    Axis* GetAxis(uint8_t axisIndex) override;

    bool SetTargetPosition(float position, char axisLabel) override;//return if axis exists

    void StartMove(float feedrate) override;

    void HomeAxes() override;

    float GetAxisPosition(char axisLabel) override;

    float GetEffectorPosition(char axisLabel) override;

    char GetEffectorAxisLabel(uint8_t axisIndex) override;

};

#include "CoreXY.tpp"
