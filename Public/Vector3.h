#ifndef OC_VECTOR3
#define OC_VECTOR3

#include "Vector.h"

struct Vector3 : public Vector<3>
{
	Vector3()
	{
	}

	Vector3(float inX, float inY, float inZ)
	: TVector {inX, inY, inZ}
	{
	}

	explicit Vector3(float k)
	: TVector(k)
	{
	}

	explicit Vector3(const TFloat& rhs)
	: TVector(rhs)
	{
	}

	Vector3(const Vector3& other)
	: TVector {other[0], other[1], other[2]}
	{
	}

	Vector3& operator=(const Vector3& other)
	{
		TVector::operator=(other);
		return *this;
	}

	float x() const { return data[0]; }
	float y() const { return data[1]; }
	float z() const { return data[2]; }
	float& x() { return data[0]; }
	float& y() { return data[1]; }
	float& z() { return data[2]; }

	static Vector3 crossProduct(const Vector3& a, const Vector3& b)
	{
		return Vector3((a.y()*b.z() - a.z()*b.y()), -(a.x()*b.z() - a.z()*b.x()), (a.x()*b.y() - a.y()*b.x()));
	}

	Vector3 operator*(float k) const { return Vector3(TVector::operator*(k)); }
	Vector3 operator+(const Vector3& rhs) const { return Vector3(TVector::operator+(rhs)); }
	Vector3 operator-(const Vector3& rhs) const { return Vector3(TVector::operator-(rhs)); }
	Vector3 operator/(float k) const { return Vector3(TVector::operator/(k)); }

	Vector3& operator*=(float k) { TVector::operator*=(k); return *this; }
	Vector3& operator/=(float k) { TVector::operator/=(k); return *this; }
	Vector3& operator+=(const Vector3& rhs) { TVector::operator+=(rhs); return *this; }
	Vector3& operator-=(const Vector3& rhs) { TVector::operator+=(rhs); return *this; }

	Vector3& operator-() { return Vector3(TVector::operator-());}

	Vector3 getNormal() const { return Vector3(TVector::getNormal()); }
	Vector3& normalize() { TVector::normalize(); return *this; }
};

inline Vector3 operator*(float lhs, const Vector3& rhs)
{
	return rhs * lhs;
}

#endif