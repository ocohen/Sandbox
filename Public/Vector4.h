#ifndef OC_VECTOR4
#define OC_VECTOR4

#include "VectorOps.h"
#include "EqualOps.h"
#include "Float4.h"

struct Vector4 : public VectorOps<Vector4>, public EqualOps<Vector4>
{
    float x;
    float y;
    float z;
    float w;

    enum { order = 4 };

	Vector4(){}
	Vector4(float inX, float inY, float inZ, float inW) : x(inX), y(inY), z(inZ), w(inW){}
	Vector4(const Vector4& other) : x(other.x), y(other.y), z(other.z), w(other.w){}
	explicit Vector4(float k) : x(k), y(k), z(k), w(k){}
    explicit Vector4(const Float4& other) : x(other.x), y(other.y), z(other.z), w(other.w){}

	Vector4& operator=(const Vector4& other)
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
