//handles list of gcode command objects
//iterates through list

#include "..\ProtoTracer\Utils\Math\Mathematics.h"

#include "Axis.h"
#include "..\lib\ProtoTracer\Utils\Controls\PID.h"

template<size_t axisCount>
class PathPlanner {
private:

    Axis<axisCount>* axes[axisCount];
    PID pid[axisCount];
    uint8_t currentAxes = 0;
    uint8_t controlAxis = 0;
    elapsedMicros dTMicro = 0;

public:
    PathPlanner();

    void AddAxis(Axis<axisCount>* axis);

    void DetermineControlAxis(float feedrate);// Interpolate axes based on limits, current velocity, target velocity, current position, target position, etc

    bool Update();
};

template<size_t axisCount>
PathPlanner<axisCount>::PathPlanner(){}

template<size_t axisCount>
void PathPlanner<axisCount>::AddAxis(Axis<axisCount>* axis){
    if (currentAxes < axisCount){
        this->axes[currentAxes] = axis;

        pid[currentAxes] = PID(1.0f, 0.0f, 0.0f);

        currentAxes++;
    }
}

template<size_t axisCount>
void PathPlanner<axisCount>::DetermineControlAxis(float feedrate){
    float maxTravelTime = 0.0f;
    
    //Calculate adjusted max feedrate, clamp feedrate
    for (int i = 0; i < currentAxes; i++){
        pid[i].Reset();// Reset PIDs before controlling

        feedrate = Mathematics::Constrain(feedrate, axes[i]->GetAxisLimits().minVelocity, axes[i]->GetAxisLimits().maxVelocity);// Clamp to axis feedrate
    }

    //Calculate single axis ideal velocity
    for (int i = 0; i < currentAxes; i++){
        axes[i]->SetTargetVelocity(feedrate);// Initial modification, then needs scaled based on position targets
    }

    //Calculate longest axis for travel movement
    for (int i = 0; i < currentAxes; i++){
        float travelTime = axes[i]->CalculateTravelTime();

        if(travelTime > maxTravelTime){// Time to travel from current position to target position, based on target speed
            maxTravelTime = travelTime;
            controlAxis = i;
        }
    }

    // Calculate several iterations to improve estimate
    for (int j = 0; j < 3; j++){
        //Scale target velocities on each axis to finish at the same time
        for (int i = 0; i < currentAxes; i++){
            // Use time to travel to position
            float travelDeltaVelocityRatio = axes[i]->CalculateTravelTime() / maxTravelTime;// Ratio of time deltas from current axis time to max axis time

            axes[i]->SetTargetVelocity(axes[i]->GetTargetVelocity() * travelDeltaVelocityRatio);// Approximation
        }
    }

    dTMicro = 0;
}

template<size_t axisCount>
bool PathPlanner<axisCount>::Update(){
    float dT = float(dTMicro) / 1000000.0f;
    bool determineComplete = !Mathematics::IsClose(axes[controlAxis]->GetCurrentPosition(), axes[controlAxis]->GetTargetPosition(), 0.01f);// First axis check if not complete

    for(int i = 1; i < currentAxes; i++){
        determineComplete |= !Mathematics::IsClose(axes[i]->GetCurrentPosition(), axes[i]->GetTargetPosition(), 0.01f);// Combine other axes if not complete
    }

    for(int i = 0; i < currentAxes; i++){
        axes[i]->Update();
    }

    //master axis ratio of completion
    float controlMoveCompletion = Mathematics::Map(axes[controlAxis]->GetCurrentPosition(), axes[controlAxis]->GetPreviousTargetPosition(), axes[controlAxis]->GetTargetPosition(), 0.0f, 1.0f);

    for(int i = 0; i < currentAxes; i++){
        if (i == controlAxis) continue;// Skip updating dynamic control if control axis

        //determine where the axis should be according to the master axis
        float moveCompletion = Mathematics::Map(axes[i]->GetCurrentPosition(), axes[i]->GetPreviousTargetPosition(), axes[i]->GetTargetPosition(), 0.0f, 1.0f);

        float currentVelocity = axes[i]->GetCurrentVelocity();

        float pidOutput = pid[i].Calculate(controlMoveCompletion, moveCompletion, dT);

        axes[i]->SetCurrentVelocity(currentVelocity + pidOutput);

        //input control to pid is difference between current

        //Serial.print(controlMoveCompletion - moveCompletion, 4); Serial.print('\n');
    }

    dTMicro = 0;

    return determineComplete;
}