# **AetherControl: Motion Control and GCODE Processor**
AetherControl is a versatile and dynamic motion control software designed to manage multiple axes, capable of handling high step frequencies up to the MHz range. It is primarily designed for the Teensy 4.0/4.1 microcontrollers, supporting both CoreXY and Cartesian kinematics with potential for expansion.

## Features:
- Multi-Axis Control: Supports single-axis and multiple-axis control, up to the limit of your GPIO capabilities.
- High Frequency: Handles step frequencies in the MHz range, with around 1MHz as the default.
- Kinematics Support: Built-in support for CoreXY and Cartesian kinematics, with easy extensibility for additional kinematics.
- GCODE Commands: Listens on serial for GCODE commands.
- Configurable and Dynamic: Designed to be more dynamic and configurable, allowing for use on any type of CNC machine.

Installation
To get started with AetherControl, clone the repository to your local machine:

```bash
git clone https://github.com/coelacant1/AetherControl.git
```

## Usage
Here is an example of a CoreXY implementation for a Pick and Place machine that can travel at 850mm/s with an acceleration of 20m/s².

## Example Code: main.cpp
```cpp
#include "Implementation/PickPlace.h"

Implementation* control = new PickPlace();

void setup() {
    control->SetSerialInterface(&Serial, 115200);

    control->Initialize();

    delay(1000);
    
    while(!Serial){}

    control->PrintInformation();

    SerialHandler::SendCommandAsk();
}

void loop() {
    if (control->IsCommandAvailable()){
        control->ExecuteCommand();

        delay(1);
    }
}
```

## Example Code: PickPlace.cpp
```cpp
#pragma once

#include "Axis.h"
#include "CoreXY.h"
#include "PathPlanner.h"
#include "PulseControl.h"
#include "WS2812B.h"
#include "Implementation.h"

class PickPlace : public Implementation{
private:
    AxisConstraints axisLimitA = AxisConstraints('A', 0.0f, 1000.0f, 0.01f, 1000.0f, 50000.0f, 20000.0f, 160.0f);
    AxisConstraints axisLimitB = AxisConstraints('B', 0.0f, 1000.0f, 0.01f, 1000.0f, 50000.0f, 20000.0f, 160.0f);

    PulseControl<2> pulseControl = PulseControl<2>();
    CoreXY<2> coreXY = CoreXY<2>(18, 16, true, true, 270, 415);

    Axis axisA = Axis(&pulseControl, &axisLimitA, 32, 31, 30);
    Axis axisB = Axis(&pulseControl, &axisLimitB, 8, 7, 6);

    WS2812B<20> wsLEDs = WS2812B<20>(19);

public:
    PickPlace() : Implementation(GCode(&coreXY)){}

    void Initialize() override;
    void PrintInformation() override;

};

void PickPlace::Initialize(){
    //initialize hardware
    axisA.Initialize();
    axisB.Initialize();

    axisA.Disable();
    axisB.Disable();

    coreXY.AddAxis(&axisA, IKinematics::A);
    coreXY.AddAxis(&axisB, IKinematics::B);
    
    wsLEDs.Initialize();
    wsLEDs.SetColor(0, 0, 0);
    wsLEDs.Update();

    pulseControl.Initialize();
    pulseControl.Enable();

    axisA.InvertDirection(true);
    axisB.InvertDirection(true);

    gCode.AddLEDs(&wsLEDs);
}
```

## Currently Supported GCODE Commands
- G0/G1: Linear move
- G4: Dwell
- G28: Homing
- M17: Enable motors
- M18: Disable motors
- M42: Switch I/O pin
- M150: Set LED color

## Contribution
Contributions are welcome! Please feel free to submit a Pull Request.

If you would like to add to contribute, please follow these steps:
- Fork the repository on GitHub.
- Commit your changes (git commit -m 'Add some YourFeature').
- Push to the branch (git push origin main).
- Submit a pull request through the GitHub website.

## License
AetherControl is licensed under the GPLv3 License.
