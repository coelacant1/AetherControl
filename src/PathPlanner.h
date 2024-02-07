//handles list of gcode command objects
//iterates through list

#include "..\ProtoTracer\Utils\Math\Mathematics.h"

#include "Axis.h"

template<size_t axisCount>
class PathPlanner {
private:

    Axis<axisCount>* axes[axisCount];
    uint8_t currentAxes = 0;
    long millisExecutionTime = 1000;//how long to lock the process
    long lastExecutionMillis;// Will break after 49.71 days of on time, implement custom timer to fix

public:
    PathPlanner();

    void AddAxis(Axis<axisCount>* axis);

    void Update(float feedrate);// Interpolate axes based on limits, current velocity, target velocity, current position, target position, etc
};

template<size_t axisCount>
PathPlanner<axisCount>::PathPlanner(){}

template<size_t axisCount>
void PathPlanner<axisCount>::AddAxis(Axis<axisCount>* axis){
    if (currentAxes < axisCount){
        this->axes[currentAxes] = axis;

        currentAxes++;
    }
}

template<size_t axisCount>
void PathPlanner<axisCount>::Update(float feedrate){
    float maxTravelTime = 0.0f;

    //Calculate single axis ideal velocity
    for (int i = 0; i < currentAxes; i++){
        axes[i]->SetTargetVelocity(Mathematics::Constrain(feedrate, axes[i]->GetAxisLimits().minVelocity, axes[i]->GetAxisLimits().maxVelocity));// Initial modification, then needs scaled based on position targets

    }

    for (int j = 0; j < 10; j++){
        
        //Calculate single axis ideal velocity
        for (int i = 0; i < currentAxes; i++){
            maxTravelTime = Mathematics::Max(maxTravelTime, axes[i]->CalculateTravelTime());// Time to travel from current position to target position, based on target speed
        }

        //Scale target velocities on each axis to finish at the same time
        for (int i = 0; i < currentAxes; i++){
            // Use time to travel to position
            float travelDeltaVelocityRatio = axes[i]->CalculateTravelTime() / maxTravelTime;// Ratio of time deltas from current axis time to max axis time

            axes[i]->SetTargetVelocity(axes[i]->GetTargetVelocity() * travelDeltaVelocityRatio);// Approximation

            Serial.print(travelDeltaVelocityRatio, 5); Serial.print('\t');
        }
        
        Serial.print('\n');
    }
}