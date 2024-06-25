#pragma once

#include <Arduino.h>

class IWS2812B {
public:
    virtual void Initialize() = 0;
    virtual void SetColor(uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual void SetLEDColor(uint8_t ledNum, uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual void Update() = 0;
};
