#pragma once

template<size_t axisCount>
PulseControl<axisCount>* PulseControl<axisCount>::instances[maxInstances] = {nullptr};

template<size_t axisCount>
size_t PulseControl<axisCount>::instanceCount = 0;

template<size_t axisCount>
PulseControl<axisCount>::PulseControl(){
    if (instanceCount < maxInstances) instances[instanceCount++] = this;

    for (uint8_t i = 0; i < axisCount; i++){
        currentPositionSteps[i] = 0;
        direction[i] = 0;
        invertDirection[i] = 0;
        currentPositionSteps[i] = 0;
        targetPositionSteps[i] = 0;
        frequencyCounter[i] = 0;
        frequencyTarget[i] = 0;
        stepped[i] = 0;

        avgFilt[i] = RunningAverageFilter<10>(0.1f);
    }
}

template<size_t axisCount>
void PulseControl<axisCount>::Initialize(){}

template<size_t axisCount>
void PulseControl<axisCount>::SetPins(uint8_t instanceNumber, uint8_t stepPin, uint8_t dirPin) {
    noInterrupts();
    PulseControl::stepPin[instanceNumber] = stepPin;
    PulseControl::dirPin[instanceNumber] = dirPin;
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::SetDirection(uint8_t instanceNumber, bool direction) {
    noInterrupts();
    PulseControl::direction[instanceNumber] = direction;
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::SetTargetPosition(uint8_t instanceNumber, long targetPosition) {
    noInterrupts();

    targetPositionSteps[instanceNumber] = targetPosition;
    
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::SetFrequency(uint8_t instanceNumber, long microseconds) {
    noInterrupts();
    frequencyTarget[instanceNumber] = microseconds;
    interrupts();
}

template<size_t axisCount>
long PulseControl<axisCount>::GetTargetPosition(uint8_t instanceNumber){
    long temp;

    noInterrupts();
    temp = targetPositionSteps[instanceNumber];
    interrupts();

    return temp;
}

template<size_t axisCount>
long PulseControl<axisCount>::GetCurrentPosition(uint8_t instanceNumber){
    long temp;

    noInterrupts();
    temp = currentPositionSteps[instanceNumber];
    interrupts();

    return temp;
}

template<size_t axisCount>
void PulseControl<axisCount>::SetCurrentPosition(uint8_t instanceNumber, long steps){
    noInterrupts();
    PulseControl::currentPositionSteps[instanceNumber] = steps;//currentPositionSteps;
    interrupts();
}

template<size_t axisCount>
void PulseControl<axisCount>::InvertDirection(uint8_t instanceNumber, bool invert) {
    noInterrupts();
    PulseControl::invertDirection[instanceNumber] = invert;
    interrupts();
}


template<size_t axisCount>
void PulseControl<axisCount>::AutoStepControl(){// Controlled by interval timer
    for(uint8_t i = 0; i < axisCount; i++){
        if (frequencyCounter[i] == 9 && stepped[i]){//10 micros
            digitalWrite(stepPin[i], LOW);// Step off

            stepped[i] = false;
        }

        if(frequencyCounter[i] >= frequencyTarget[i]){
            frequencyCounter[i] = 0;
            
            if(currentPositionSteps[i] < targetPositionSteps[i]){
                currentPositionSteps[i]++;


                digitalWrite(dirPin[i], invertDirection[i] ? !direction[i] : direction[i]);// Set Direction
                digitalWrite(stepPin[i], HIGH);// Step on

                stepped[i] = true;
            }
            else if (currentPositionSteps[i] > targetPositionSteps[i]){
                currentPositionSteps[i]--;

                
                digitalWrite(dirPin[i], invertDirection[i] ? direction[i] : !direction[i]);// Set Direction
                digitalWrite(stepPin[i], HIGH);// Step on
                
                stepped[i] = true;
            }
        }
        
        frequencyCounter[i]++;
    }
}

template<size_t axisCount>
void PulseControl<axisCount>::StaticTimerCallback(){
    // Call the member function for each registered instance
    if (instances[0] != nullptr) instances[0]->AutoStepControl();
    if (instances[1] != nullptr) instances[1]->AutoStepControl();
    if (instances[2] != nullptr) instances[2]->AutoStepControl();
    if (instances[3] != nullptr) instances[3]->AutoStepControl();
    if (instances[4] != nullptr) instances[4]->AutoStepControl();
    if (instances[5] != nullptr) instances[5]->AutoStepControl();
    if (instances[6] != nullptr) instances[6]->AutoStepControl();
    if (instances[7] != nullptr) instances[7]->AutoStepControl();
}

template<size_t axisCount>
void PulseControl<axisCount>::Enable(){
    pulseTimer.begin(StaticTimerCallback, 1);//1MHz max update rate
}

template<size_t axisCount>
void PulseControl<axisCount>::Disable(){
    pulseTimer.end();
}
