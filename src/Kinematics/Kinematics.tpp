#pragma once

template<size_t axisCount>
void Kinematics<axisCount>::AddAxis(Axis* axis, AxisLabel axisLabel){
    if (currentAxes < axisCount){
        this->axes[currentAxes] = axis;
        this->axisLabel[currentAxes] = axisLabel;
        
        pathPlanner.AddAxis(axis);

        currentAxes++;
    }
}

template<size_t axisCount>
Axis* Kinematics<axisCount>::GetAxis(uint8_t axisIndex){
    return axes[axisIndex];
}

template<size_t axisCount>
uint8_t Kinematics<axisCount>::GetAxisCount(){
    return currentAxes;
}
