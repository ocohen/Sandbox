#ifndef OC_VECTOR4
#define OC_VECTOR4

#include "Vector.h"

struct Vector4 : public Vector<4>
{
	Vector4()
	{
	}

	Vector4(float inX, float inY, float inZ, float inW)
	: TVector {inX, inY, inZ, inW}
	{
	}

	Vector4(const Vector4& other)
	: TVector {other[0], other[1], other[2], other[3]}
	{
	}

	explicit Vector4(float k)
	: TVector(k)
	{
	}

	explicit Vector4(const NFloat<4>& rhs)
	: TVector(rhs)
	{
	}

	Vector4& operator=(const Vector4& other)
	{
		TVector::operator=(other);
		return *this;
	}

	float x() const { return data[0]; }
	float y() const { return data[1]; }
	float z() const { return data[2]; }
	float w() const { return data[3]; }
	float& x() { return data[0]; }
	float& y() { return data[1]; }
	float& z() { return data[2]; }
	float& w() { return data[3]; }

	Vector4 operator*(float k) const { return Vector4(TVector::operator*(k)); }
	Vector4 operator+(const Vector4& rhs) const { return Vector4(TVector::operator+(rhs)); }
	Vector4 operator-(const Vector4& rhs) const { return Vector4(TVector::operator-(rhs)); }
	Vector4 operator/(float k) const { return Vector4(TVector::operator/(k)); }

	Vector4& operator*=(float k) { TVector::operator*=(k); return *this; }
	Vector4& operator/=(float k) { TVector::operator/=(k); return *this; }
	Vector4& operator+=(const Vector4& rhs) { TVector::operator+=(rhs); return *this; }
	Vector4& operator-=(const Vector4& rhs) { TVector::operator+=(rhs); return *this; }

	Vector4& operator-() { return Vector4(TVector::operator-()); }

	Vector4 getNormal() const { return Vector4(TVector::getNormal()); }
	Vector4& normalize() { TVector::normalize(); return *this; }
};

inline Vector4 operator*(float lhs, const Vector4& rhs)
{
	return rhs * lhs;
}

#endif