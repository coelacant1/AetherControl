#pragma once

#include "IWS2812B.h"

// Simple bit bang WS2812B control, no background operations for updating
template<size_t ledCount>
class WS2812B : public IWS2812B {
private:
    uint8_t ledData[ledCount * 3];
    uint8_t pin;

    void SendBit(bool bit);
    void SendByte(uint8_t byte);
    void SendReset();

public:
    WS2812B(uint8_t pin);
    
    void Initialize() override;
    void SetColor(uint8_t r, uint8_t g, uint8_t b) override;
    void SetLEDColor(uint8_t ledNum, uint8_t r, uint8_t g, uint8_t b) override;
    void Update() override;
};

#include "WS2812B.tpp"