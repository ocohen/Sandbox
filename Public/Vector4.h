#ifndef OC_VECTOR4
#define OC_VECTOR4

#include "Vector.h"

struct Vector4 : public Vector<4>
{
	float& x = Vector<4>::data[0];
	float& y = Vector<4>::data[1];
	float& z = Vector<4>::data[2];
	float& w = Vector<4>::data[3];

	Vector4()
	{
	}

	Vector4(float x, float y, float z, float w)
	: Vector<4>(x,y,z,w)
	{
	}

	Vector4(float k)
	: Vector<4>(k)
	{
	}

	Vector4(const Vector<4>& rhs)
	: Vector<4>(rhs)
	{
	}
};

#endif