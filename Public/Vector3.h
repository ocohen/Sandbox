#ifndef OC_VECTOR3
#define OC_VECTOR3

#include "Vector.h"

struct Vector3 : public Vector<3>
{
	float& x = Vector<3>::data[0];
	float& y = Vector<3>::data[1];
	float& z = Vector<3>::data[2];

	Vector3()
	{
	}

	Vector3(float x, float y, float z)
	: Vector<3>(x,y,z)
	{
	}

	Vector3(float k)
	: Vector<3>(k)
	{
	}

	Vector3(const Vector<3>& rhs)
	: Vector<3>(rhs)
	{
	}

	static Vector3 crossProduct(const Vector3& a, const Vector3& b)
	{
		return Vector3((a.y*b.z - a.z*b.y) - (a.x*b.z - a.z*b.x) + (a.x*b.y - a.y*b.x));
	}
};

#endif