#pragma once

// Class for implementation of function calls for gcode commands
class GCode {
private:

public:
    void G0();// Rapid move
    void G1();// Linear move
    void G28();// Home all non-relative axes

    void M150();// Set LED color
    
};
