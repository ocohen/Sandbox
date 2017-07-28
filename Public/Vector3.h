#ifndef OC_VECTOR3
#define OC_VECTOR3

#include "VectorOps.h"
#include "EqualOps.h"
#include "Float3.h"

struct Vector3 : public VectorOps<Vector3>, public EqualOps<Vector3>
{
    float x;
    float y;
    float z;
    enum{ order=3 };

    Vector3(){}
    Vector3(float inX, float inY, float inZ): x(inX), y(inY), z(inZ){}
    Vector3(const Vector3& rhs) : x(rhs.x), y(rhs.y), z(rhs.z){}
    explicit Vector3(float k) : x(k), y(k), z(k) {}
    explicit Vector3(const Float3& other) : x(other.x), y(other.y), z(other.z){}


    Vector3& operator=(const Vector3& rhs)
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

	static Vector3 crossProduct(const Vector3& a, const Vector3& b)
	{
        return Vector3(a.y*b.z - a.z*b.y, -(a.x*b.z - a.z*b.x), a.x*b.y - a.y*b.x);
	}
};
#endif
