//#include "Implementation/PickPlace.h"
//#include "Implementation/PickPlaceHead.h"
#include "Implementation/StewartPlatform.h"

//PickPlace pickPlace = PickPlace();
Implementation* control = new StewartPlatform();

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
