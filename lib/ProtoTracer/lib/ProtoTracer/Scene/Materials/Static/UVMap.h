#pragma once

#include "Image.h"
#include "../../../Utils/Math/Mathematics.h"

class UVMap : public Image {
public:
    UVMap(const uint8_t* data, const uint8_t* rgbColors, uint16_t xPixels, uint16_t yPixels, uint8_t colors);

    RGBColor GetRGB(const Vector3D& position, const Vector3D& normal, const Vector3D& uvw) override;
};
