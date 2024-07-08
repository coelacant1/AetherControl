#pragma once

template<size_t axisCount>
PathPlanner<axisCount>::PathPlanner() {}

template<size_t axisCount>
void PathPlanner<axisCount>::AddAxis(Axis* axis){
    if (currentAxes < axisCount){
        this->axes[currentAxes] = axis;

        currentAxes++;
    }
}

template<size_t axisCount>
Axis* PathPlanner<axisCount>::GetAxis(uint8_t axisIndex){
    return this->axes[axisIndex];
}

template<size_t axisCount>
uint8_t PathPlanner<axisCount>::GetAxisCount(){
    return currentAxes;
}

template<size_t axisCount>
void PathPlanner<axisCount>::CalculateLimits(float feedrate){
    //Calculate the limits for max velocity of all axes, acceleration
    float axisDistance[axisCount];
    acceleration = 1000000.0f;// Absurdly high acceleration
    
    //Calculate adjusted max feedrate, clamp feedrate
    for (int i = 0; i < currentAxes; i++){
        startPosition[i] = axes[i]->GetCurrentPosition();
        endPosition[i] = axes[i]->GetTargetPosition();

        axisDistance[i] = fabsf(endPosition[i] - startPosition[i]);

        feedrate = Mathematics::Constrain(feedrate, axes[i]->GetAxisConstraints()->GetMinVelocity(), axes[i]->GetAxisConstraints()->GetMaxVelocity());// Clamp to axis feedrate
        acceleration = Mathematics::Min(acceleration, axes[i]->GetAxisConstraints()->GetAcceleration());
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
    float dT = 0.0001f;

    if (!newCommand) dT = float(sinceUpdate) / 1000000.0f;
    else velocity = 0.0f;

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

    ratio = Mathematics::Constrain(ratio + velocity * dT, 0.001f, 1.0f);

    bool axesFinished = Mathematics::IsClose(ratio, 1.0f, 0.001f);

    for (int i = 0; i < currentAxes; i++){
        // Last steps per second from previous window
        float targetPosition = Mathematics::Map(ratio, 0.0f, 1.0f, startPosition[i], endPosition[i]);// Starting position must always be slightly greater than current position

        if (Mathematics::IsClose(ratio, 1.0f, 0.001f)) targetPosition = endPosition[i];// Finalize end target at slowest velocity for last move update

        if(newCommand){// Prevents initial impulse from rapid change in velocity estimation
            axes[i]->SetTargetPosition(targetPosition);
            axes[i]->SetTargetPosition(targetPosition);
        }

        float velocity = fabsf(targetPosition - axes[i]->GetPreviousTargetPosition()) / (dT * 2.0f);//

        velocity = Mathematics::Constrain(velocity, axes[i]->GetAxisConstraints()->GetMinVelocity(), axes[i]->GetAxisConstraints()->GetMaxVelocity());
        
        axes[i]->SetCurrentVelocity(velocity);
        axes[i]->SetTargetPosition(targetPosition);

        axesFinished = axesFinished && axes[i]->GetAbsoluteCurrentPosition() == axes[i]->GetAbsoluteTargetPosition();
    }

    newCommand = false;

    return !axesFinished;
}
