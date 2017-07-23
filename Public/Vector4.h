#ifndef OC_VECTOR4
#define OC_VECTOR4

#include "NFloat.h"

struct Vector4 : public NFloat<4>
{
	float& x = NFloat<4>::data[0];
	float& y = NFloat<4>::data[1];
	float& z = NFloat<4>::data[2];
	float& w = NFloat<4>::data[3];

	Vector4()
	{
	}

	Vector4(float x, float y, float z, float w)
	: NFloat<4>(x,y,z,w)
	{
	}

	explicit Vector4(float k)
	: NFloat<4>(k)
	{
	}

	explicit Vector4(const NFloat<4>& rhs)
	: NFloat<4>(rhs)
	{
	}

	Vector4& operator=(const Vector4& rhs){ NFloat<4>::operator=(rhs); return *this; }

	NFloat<4> toNFloat() const
	{
		return NFloat<4>(x, y, z, w);
	}

	Vector4 operator*(float k) const { return Vector4(NFloat<4>::operator*(k)); }
	Vector4 operator+(const Vector4& rhs) const { return Vector4(NFloat<4>::operator+(rhs)); }
	Vector4 operator-(const Vector4& rhs) const { return Vector4(NFloat<4>::operator-(rhs)); }
	Vector4 operator/(float k) const { return Vector4(NFloat<4>::operator/(k)); }

	Vector4& operator*=(float k) { NFloat<4>::operator*=(k); return *this; }
	Vector4& operator/=(float k) { NFloat<4>::operator/=(k); return *this; }
	Vector4& operator+=(const Vector4& rhs) { NFloat<4>::operator+=(rhs); return *this; }
	Vector4& operator-=(const Vector4& rhs) { NFloat<4>::operator+=(rhs); return *this; }
	float dotProduct(const Vector4& rhs) const { return NFloat<4>::dotProduct(rhs); }
	static float dotProduct(const Vector4& a, const Vector4& b) { return NFloat<4>::dotProduct(a, b); }

	Vector4 getNormal() const { return Vector4(NFloat<4>::getNormal()); }
	Vector4& normalize() { NFloat<4>::normalize(); return *this; }

	using NFloat<4>::length;
	using NFloat<4>::length2;
	static bool isNearlyEqual(const Vector4& a, const Vector4& b) { return NFloat<4>::isNearlyEqual(a, b); }
};

#endif