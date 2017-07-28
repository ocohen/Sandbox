#ifndef OC_FLOAT4
#define OC_FLOAT4

#include "FloatOps.h"
#include "EqualOps.h"

struct Float4 : public FloatOps<Float4>, public EqualOps<Float4>
{
    float x;
    float y;
    float z;
    float w;

    enum { order = 4 };

    Float4(){}
    Float4(float inX, float inY, float inZ, float inW) : x(inX), y(inY), z(inZ), w(inW){}
    Float4(const Float4& other) : x(other.x), y(other.y), z(other.z), w(other.w){}
    explicit Float4(float k) : x(k), y(k), z(k), w(k){}

    Float4& operator=(const Float4& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
        return *this;
    }

    float operator[](int index) const { const float* vals = &x; return vals[index]; }
    float& operator[](int index) { float* vals = &x; return vals[index]; }
};
#endif
