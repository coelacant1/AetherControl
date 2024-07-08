#include "GlobalVariables.h"

bool GlobalVariables::relativeMode = false;

const bool GlobalVariables::printCharacterStream = false;

const float GlobalVariables::homingSpeedPrimary = 40.0f;
const float GlobalVariables::homingSpeedSecondary = 10.0f;
const float GlobalVariables::homingDistanceOffset = 100.0f;
const float GlobalVariables::homingDistanceBackoff = 5.0f;

const char GlobalVariables::firmwareVersion[] = "1.0.0"; 
const char GlobalVariables::firmwareName[] = "AetherControl"; 
const char GlobalVariables::firmwareSourceURL[] = "https://github.com/coelacant1/AetherControl"; 