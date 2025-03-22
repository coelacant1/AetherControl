#include "Implementation/StewartPlatform.h"
//#include "Implementation/PickPlaceHead.h"

Implementation* control = new StewartPlatform();
int count = 0;

void setup() {
    control->SetSerialInterface(&Serial, 115200);

    control->Initialize();

    delay(1000);
    
    while(!Serial){}

    control->PrintInformation();
}

void loop() {
    if (control->IsCommandAvailable()){
        control->ExecuteCommand();

        delay(1);
        
        control->PrintInformation();
    }
}
