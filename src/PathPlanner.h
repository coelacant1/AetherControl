//handles list of gcode command objects
//iterates through list

#include "..\ProtoTracer\Utils\Math\Mathematics.h"

#include "Axis.h"
#include "..\lib\ProtoTracer\Utils\Controls\PID.h"

template<size_t axisCount>
class PathPlanner {
private:

    Axis<axisCount>* axes[axisCount];
    float startPosition[axisCount];
    float endPosition[axisCount];
    uint8_t currentAxes = 0;
    elapsedMicros sinceUpdate;
    PID pid[axisCount];

    float velocity = 0.0f;
    float targetVelocity = 0.0f;
    float acceleration = 0.0f;
    float ratio = 0.0f;

    bool newCommand = true;

public:
    PathPlanner();

    void AddAxis(Axis<axisCount>* axis);

    void CalculateLimits(float feedrate);

    //void DetermineControlAxis(float feedrate);// Interpolate axes based on limits, current velocity, target velocity, current position, target position, etc

    bool Update();
};

template<size_t axisCount>
PathPlanner<axisCount>::PathPlanner(){}

template<size_t axisCount>
void PathPlanner<axisCount>::AddAxis(Axis<axisCount>* axis){
    if (currentAxes < axisCount){
        this->axes[currentAxes] = axis;

        pid[currentAxes] = PID(100.0f, 0.0f, 0.0f);

        currentAxes++;
    }
}

template<size_t axisCount>
void PathPlanner<axisCount>::CalculateLimits(float feedrate){
    //Calculate the limits for max velocity of all axes, acceleration
    float axisDistance[axisCount];
    acceleration = 1000000.0f;// Absurdly high acceleration
    
    //Calculate adjusted max feedrate, clamp feedrate
    for (int i = 0; i < currentAxes; i++){
        pid[i].Reset();

        startPosition[i] = axes[i]->GetCurrentPosition();
        endPosition[i] = axes[i]->GetTargetPosition();

        axisDistance[i] = fabsf(endPosition[i] - startPosition[i]);

        feedrate = Mathematics::Constrain(feedrate, axes[i]->GetAxisLimits().minVelocity, axes[i]->GetAxisLimits().maxVelocity);// Clamp to axis feedrate
        acceleration = Mathematics::Min(acceleration, axes[i]->GetAcceleration());
    }

    float distance = 0.0f;

    for (int i = 0; i < currentAxes; i++){
        distance += axisDistance[i] * axisDistance[i];
    }

    distance = Mathematics::Sqrt(distance);// Euclidian distance
    
    targetVelocity = feedrate / distance;// Normalize target feedrate to scale of 0.0 to 1.0
    acceleration = acceleration / distance;// Normalize acceleration to scale of 0.0 to 1.0
    
    ratio = 0.0f;
    newCommand = true;

    sinceUpdate = 0;
}

template<size_t axisCount>
bool PathPlanner<axisCount>::Update(){
    float dT = 0.0f;

    if (!newCommand){
        dT = float(sinceUpdate) / 1000000.0f;
    }
    else{
        newCommand = false;
        velocity = 0.0f;
    }

    sinceUpdate = 0;

    float velocityChange = acceleration * dT;// Desired change in velocity based on acceleration
    float remainingDistance = fabsf(1.0f - ratio);// Acceleration or deceleration phase
    float decelerationDistance = (velocity * velocity) / (2.0f * acceleration);// Stopping distance

    if (ratio < 1.0f) {// Move forward
        if (remainingDistance > decelerationDistance && fabsf(velocity) < targetVelocity){
            velocity += velocityChange;// Accelerate
        }
        else {
            velocity -= velocityChange;// Decelerate
        }
    }

    if (remainingDistance < 0.001f) velocity = 0;

    ratio = Mathematics::Constrain(ratio + velocity * dT, 0.0f, 1.0f);

    for (int i = 0; i < currentAxes; i++){
        //last steps per second from previous window
        //dT by steps / previous axis velocity
        float controlPosition = Mathematics::Map(ratio, 0.0f, 1.0f, startPosition[i], endPosition[i]);
        float frequency = (fabsf(controlPosition - axes[i]->GetControlPreviousPosition()) / dT) * axes[i]->GetStepsPerMillimeter();
        
        axes[i]->SetControlFrequency(frequency / 2.0f);//fabsf(pid[i].Calculate(axes[i]->GetTargetPosition(), axes[i]->GetCurrentPosition(), dT)));//20000);// Counter intuitive, but target position will only allow minimal movement as long as this is updated enough
        axes[i]->SetControlPosition(controlPosition);// Sets the target position along the mapped N-dimensional line
    }

    return !Mathematics::IsClose(ratio, 1.0f, 0.001f);
}
