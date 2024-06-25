#pragma once

#include "../Material.h"
#include "../../../Utils/Math/Vector2D.h"
#include "../../../Utils/Math/Mathematics.h"

class Image : public Material {
public:
    Vector2D size;
    Vector2D offset;
    float angle = 0.0f;
    float hueAngle = 0.0f;
    unsigned int xPixels = 0;
    unsigned int yPixels = 0;
    const uint8_t* data;
    const uint8_t* rgbColors;
    uint8_t colors;

    Image(const uint8_t* data, const uint8_t* rgbColors, unsigned int xPixels, unsigned int yPixels, uint8_t colors);

    ~Image() {}

    void SetData(const uint8_t* data);

    void SetColorPalette(const uint8_t* rgbColors);

    void SetSize(Vector2D size);

    void SetPosition(Vector2D offset);

    void SetRotation(float angle);

    void SetHueAngle(float hueAngle);

    RGBColor GetRGB(const Vector3D& position, const Vector3D& normal, const Vector3D& uvw) override;
};
