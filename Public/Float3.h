#ifndef OC_FLOAT3
#define OC_FLOAT3

#include "FloatOps.h"
#include "EqualOps.h"

struct Float3 : public FloatOps<Float3>, public EqualOps<Float3>
{
    float x;
    float y;
    float z;
    enum{ order=3 };

    Float3(){}
    Float3(float inX, float inY, float inZ): x(inX), y(inY), z(inZ){}
    Float3(const Float3& rhs) : x(rhs.x), y(rhs.y), z(rhs.z){}
    explicit Float3(float k) : x(k), y(k), z(k) {}

    Float3& operator=(const Float3& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    float operator[](int index) const
    {
       const float* vals = &x;
       return vals[index]; 
    }

    float& operator[](int index)
    {
       float* vals = &x;
       return vals[index]; 
    }
};
#endif
