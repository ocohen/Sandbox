#ifndef OC_VECTOR3
#define OC_VECTOR3

#include "NFloat.h"

struct Vector3 : private NFloat<3>
{
	float& x = NFloat<3>::data[0];
	float& y = NFloat<3>::data[1];
	float& z = NFloat<3>::data[2];

	Vector3()
	{
	}

	Vector3(float x, float y, float z)
	: NFloat<3>(x,y,z)
	{
	}

	explicit Vector3(float k)
	: NFloat<3>(k)
	{
	}

	explicit Vector3(const NFloat<3>& rhs)
	: NFloat<3>(rhs)
	{
	}

	NFloat<3> toNFloat() const
	{
		return NFloat<3>(x,y,z);
	}

	Vector3& operator=(const Vector3& rhs){ NFloat<3>::operator=(rhs); return *this; }

	static Vector3 crossProduct(const Vector3& a, const Vector3& b)
	{
		return Vector3((a.y*b.z - a.z*b.y), -(a.x*b.z - a.z*b.x), (a.x*b.y - a.y*b.x));
	}

	Vector3 operator*(float k) const { return Vector3(NFloat<3>::operator*(k)); }
	Vector3 operator+(const Vector3& rhs) const { return Vector3(NFloat<3>::operator+(rhs)); }
	Vector3 operator-(const Vector3& rhs) const { return Vector3(NFloat<3>::operator-(rhs)); }
	Vector3 operator/(float k) const { return Vector3(NFloat<3>::operator/(k)); }

	Vector3& operator*=(float k){ NFloat<3>::operator*=(k); return *this; }
	Vector3& operator/=(float k) { NFloat<3>::operator/=(k); return *this; }
	Vector3& operator+=(const Vector3& rhs) { NFloat<3>::operator+=(rhs); return *this; }
	Vector3& operator-=(const Vector3& rhs) { NFloat<3>::operator+=(rhs); return *this; }
	float dotProduct(const Vector3& rhs) const { return NFloat<3>::dotProduct(rhs); }
	static float dotProduct(const Vector3& a, const Vector3& b){ return NFloat<3>::dotProduct(a,b); }

	Vector3 getNormal() const { return Vector3(NFloat<3>::getNormal()); }
	Vector3& normalize(){ NFloat<3>::normalize(); return *this; }

	using NFloat<3>::length;
	using NFloat<3>::length2;
	static bool isNearlyEqual(const Vector3& a, const Vector3& b) { return NFloat<3>::isNearlyEqual(a, b); }
};

#endif