#pragma once

#include <Arduino.h>

// Simple bit bang WS2812B control, no background operations for updating
template<size_t ledCount>
class WS2812B {
private:
    uint8_t ledData[ledCount * 3];
    uint8_t pin;

    void SendBit(bool bit);
    void SendByte(uint8_t byte);
    void SendReset();

public:
    WS2812B(uint8_t pin);
    
    void Initialize();
    void SetColor(uint8_t r, uint8_t g, uint8_t b);
    void SetLEDColor(uint8_t ledNum, uint8_t r, uint8_t g, uint8_t b);
    void Update();
};

template<size_t ledCount>
void WS2812B<ledCount>::SendBit(bool bit){
    if (bit){
        digitalWrite(pin, HIGH);
        delayNanoseconds(750);
        digitalWrite(pin, LOW);
        delayNanoseconds(200);
    }
    else{
        digitalWrite(pin, HIGH);
        delayNanoseconds(200);
        digitalWrite(pin, LOW);
        delayNanoseconds(750);
    }
}

template<size_t ledCount>
WS2812B<ledCount>::WS2812B(uint8_t pin){
    this->pin = pin;
}

template<size_t ledCount>
void WS2812B<ledCount>::Initialize(){
    pinMode(pin, OUTPUT);
}

template<size_t ledCount>
void WS2812B<ledCount>::SendByte(uint8_t byte){
    for (int i = 7; i >= 0; i--) SendBit(byte & (1 << i));
}

template<size_t ledCount>
void WS2812B<ledCount>::SendReset(){
    delayMicroseconds(50);
}

template<size_t ledCount>
void WS2812B<ledCount>::SetColor(uint8_t r, uint8_t g, uint8_t b){
    for (int i = 0; i < int(ledCount); i++){
        ledData[i * 3 + 1] = r;
        ledData[i * 3 + 0] = g;
        ledData[i * 3 + 2] = b;
    }
}

template<size_t ledCount>
void WS2812B<ledCount>::SetLEDColor(uint8_t ledNum, uint8_t r, uint8_t g, uint8_t b){
    if (ledNum > ledCount) return;

    ledData[ledNum * 3 + 1] = r;
    ledData[ledNum * 3 + 0] = g;
    ledData[ledNum * 3 + 2] = b;
}

template<size_t ledCount>
void WS2812B<ledCount>::Update(){
    noInterrupts();
    for (int i = 0; i < int(ledCount) * 3; i++) SendByte(ledData[i]);
    interrupts();
    
    SendReset();
}
