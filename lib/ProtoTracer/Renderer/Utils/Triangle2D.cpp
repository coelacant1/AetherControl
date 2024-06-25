#include "Triangle2D.h"

Triangle2D::Triangle2D() {}

Triangle2D::Triangle2D(const Vector2D& p1, const Vector2D& p2, const Vector2D& p3) {
    p1X = p1.X;
    p1Y = p1.Y;
    p2X = p2.X;
    p2Y = p2.Y;
    p3X = p3.X;
    p3Y = p3.Y;

    v0X = p2X - p1X;
    v0Y = p2Y - p1Y;
    v1X = p3X - p1X;
    v1Y = p3Y - p1Y;

    denominator = 1.0f / (v0X * v1Y - v1X * v0Y);
}

Triangle2D::Triangle2D(const Quaternion& lookDirection, Transform* camT, Triangle3D* t, Material* material) {
    this->material = material;

    if (t->hasUV) {
        this->p1UV = t->p1UV;
        this->p2UV = t->p2UV;
        this->p3UV = t->p3UV;

        this->hasUV = true;
    }

    Vector3D p1 = camT->GetRotation().Multiply(lookDirection).UnrotateVector(*t->p1 * camT->GetScale() - camT->GetPosition());
    Vector3D p2 = camT->GetRotation().Multiply(lookDirection).UnrotateVector(*t->p2 * camT->GetScale() - camT->GetPosition());
    Vector3D p3 = camT->GetRotation().Multiply(lookDirection).UnrotateVector(*t->p3 * camT->GetScale() - camT->GetPosition());

    averageDepth = (p1.Z + p2.Z + p3.Z) / 3.0f;

    p1X = p1.X;
    p1Y = p1.Y;
    p2X = p2.X;
    p2Y = p2.Y;
    p3X = p3.X;
    p3Y = p3.Y;

    v0X = p2X - p1X;
    v0Y = p2Y - p1Y;
    v1X = p3X - p1X;
    v1Y = p3Y - p1Y;

    denominator = 1.0f / (v0X * v1Y - v1X * v0Y);

    normal = t->Normal();

    t3p1 = t->p1;
    t3p2 = t->p2;
    t3p3 = t->p3;

    //for quadtree / min/max dimensions
    if (p1X < p2X){
        min.X = (p1X < p3X ? p1X : p3X);
        max.X = (p2X > p3X ? p2X : p3X);
    }
    else{
        min.X = (p2X < p3X ? p2X : p3X);
        max.X = (p1X > p3X ? p1X : p3X);
    }

    if (p1Y < p2Y){
        min.Y = (p1Y < p3Y ? p1Y : p3Y);
        max.Y = (p2Y > p3Y ? p2Y : p3Y);
    }
    else{
        min.Y = (p2Y < p3Y ? p2Y : p3Y);
        max.Y = (p1Y > p3Y ? p1Y : p3Y);
    }
}

Triangle2D::Triangle2D(Triangle3D* t) {
    averageDepth = (t->p1->Z + t->p2->Z + t->p3->Z) / 3.0f;

    p1X = t->p1->X;
    p1Y = t->p1->Y;
    p2X = t->p2->X;
    p2Y = t->p2->Y;
    p3X = t->p3->X;
    p3Y = t->p3->Y;

    v0X = p2X - p1X;
    v0Y = p2Y - p1Y;
    v1X = p3X - p1X;
    v1Y = p3Y - p1Y;

    denominator = 1.0f / (v0X * v1Y - v1X * v0Y);

    t3p1 = t->p1;
    t3p2 = t->p2;
    t3p3 = t->p3;
    
    //for quadtree / min/max dimensions
    if (p1X < p2X){
        min.X = (p1X < p3X ? p1X : p3X);
        max.X = (p2X > p3X ? p2X : p3X);
    }
    else{
        min.X = (p2X < p3X ? p2X : p3X);
        max.X = (p1X > p3X ? p1X : p3X);
    }

    if (p1Y < p2Y){
        min.Y = (p1Y < p3Y ? p1Y : p3Y);
        max.Y = (p2Y > p3Y ? p2Y : p3Y);
    }
    else{
        min.Y = (p2Y < p3Y ? p2Y : p3Y);
        max.Y = (p1Y > p3Y ? p1Y : p3Y);
    }
}

Vector2D Triangle2D::GetP1() {
    return Vector2D(p1X, p1Y);
}

Vector2D Triangle2D::GetP2() {
    return Vector2D(p2X, p2Y);
}

Vector2D Triangle2D::GetP3() {
    return Vector2D(p3X, p3Y);
}

Material* Triangle2D::GetMaterial() {
    return material;
}

bool Triangle2D::DidIntersect(const float& x, const float& y, float& u, float& v, float& w) {
    float v2lX = x - p1X;
    float v2lY = y - p1Y;

    v = (v2lX * v1Y - v1X * v2lY) * denominator;
    w = (v0X * v2lY - v2lX * v0Y) * denominator;

    if (v < 0.0f || w < 0.0f || v > 1.0f || w > 1.0f) return false;

    u = 1.0f - v - w;

    return u >= 0.0f;
}

bool Triangle2D::DidIntersect(BoundingBox2D* bbox) {
    return bbox->Overlaps(min, max);
}

//Separating Axis Theorem (SAT) algorithm
bool Triangle2D::DidIntersectSAT(BoundingBox2D* bbox) {
    if (!(bbox->GetMinimum().X < Mathematics::Max(p1X, p2X, p3X) && bbox->GetMaximum().X > Mathematics::Min(p1X, p2X, p3X))) { return false; }
    if (!(bbox->GetMinimum().Y < Mathematics::Max(p1Y, p2Y, p3Y) && bbox->GetMaximum().Y > Mathematics::Min(p1Y, p2Y, p3Y))) { return false; }

    Vector2D axes[] = {
        {-1.0f * (p2Y - p1Y), p2X - p1X},
        {-1.0f * (p3Y - p1Y), p3X - p1X}, 
        {-1.0f * (p3Y - p2Y), (p3X - p2X)}
    }; 

    Vector2D c = Vector2D((bbox->GetMinimum().X + bbox->GetMaximum().X) * 0.5f, (bbox->GetMinimum().Y + bbox->GetMaximum().Y) * 0.5f);
    Vector2D e = Vector2D((bbox->GetMaximum().X - bbox->GetMinimum().X) * 0.5f, (bbox->GetMaximum().Y - bbox->GetMinimum().Y) * 0.5f);

    float p0 = axes[0].X * (p1X - c.X) + axes[0].Y * (p1Y - c.Y);
    float p2 = axes[0].X * (p3X - c.X) + axes[0].Y * (p3Y - c.Y);

    float r = e.X * fabsf(axes[0].X) + e.Y * fabsf(axes[0].Y);

    if (Mathematics::Max(-1.0f * Mathematics::Max(p0, p2), Mathematics::Min(p0, p2)) > r)
        return false;

    p0 = axes[1].X * (p1X - c.X) + axes[1].Y * (p1Y - c.Y);
    float p1 = axes[1].X * (p2X - c.X) + axes[1].Y * (p2Y - c.Y);

    r = e.X * fabsf(axes[1].X) + e.Y * fabsf(axes[1].Y);

    if (Mathematics::Max(-1.0f * Mathematics::Max(p0, p1), Mathematics::Min(p0, p1)) > r)
        return false;

    p0 = axes[2].X * (p1X - c.X) + axes[2].Y * (p1Y - c.Y);
    p1 = axes[2].X * (p2X - c.X) + axes[2].Y * (p2Y - c.Y);

    r = e.X * fabsf(axes[2].X) + e.Y * fabsf(axes[2].Y);

    if (Mathematics::Max(-1.0f * Mathematics::Max(p0, p1), Mathematics::Min(p0, p1)) > r)
        return false;

    return true;
}

String Triangle2D::ToString() {
    return Vector2D(p1X, p1Y).ToString() + " " + Vector2D(p2X, p2Y).ToString() + " " + Vector2D(p3X, p3Y).ToString();
}
