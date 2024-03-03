#pragma once

#include "Axis.h"

class IKinematics{
public:
    enum AxisLabel {
        A = 'A',
        B = 'B',
        E = 'E',
        I = 'I',
        J = 'J',
        K = 'K',
        U = 'U',
        V = 'V',
        W = 'W',
        X = 'X',
        Y = 'Y',
        Z = 'Z'
    };
    
    virtual void AddAxis(Axis* axis, AxisLabel axisLabel) = 0;
    virtual Axis* GetAxis(char axisLabel) = 0;
    virtual uint8_t GetAxisCount() = 0;
    virtual bool SetTargetPosition(float position, char axisLabel) = 0;//return if axis exists
    virtual void StartMove(float feedrate) = 0;
    virtual void HomeAxes() = 0;

};
