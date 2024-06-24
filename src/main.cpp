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
