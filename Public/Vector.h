#ifndef OC_VECTOR
#define OC_VECTOR

#include "NFloat.h"

template <int N>
struct Vector : private NFloat<N>
{
	typedef Vector<N> TVector;

	Vector() {}

	explicit Vector(float k)
	: TFloat(k)
	{
	}

	explicit Vector(const TFloat& rhs)
	: TFloat(rhs)
	{
	}

	Vector(const Vector& other)
	: TFloat(other)
	{
	}

	template <typename... T>
	Vector(T... inData)
	: TFloat(inData...)
	{
	}

	TVector& operator=(const TVector& rhs)
	{
		TFloat::operator=(rhs);
		return *this;
	}

	const TFloat& asNFloat() const
	{
		return reinterpret_cast<const TFloat&>(*this);
	}

	TFloat& asNFloat()
	{
		return reinterpret_cast<TFloat&>(*this);
	}

	Vector operator*(float k) const { return TVector(TFloat::operator*(k)); }
	Vector operator+(const TVector& rhs) const { return TVector(TFloat::operator+(rhs)); }
	Vector operator-(const TVector& rhs) const { return TVector(TFloat::operator-(rhs)); }
	Vector operator/(float k) const { return TVector(TFloat::operator/(k)); }
	
	Vector& operator*=(float k) { TFloat::operator*=(k); return *this; }
	Vector& operator/=(float k) { TFloat::operator/=(k); return *this; }
	Vector& operator+=(const TVector& rhs) { TFloat::operator+=(rhs); return *this; }
	Vector& operator-=(const TVector& rhs) { TFloat::operator+=(rhs); return *this; }

	Vector& operator-() const { return TVector(TFloat::operator-()); }

	static float dotProduct(const TVector& a, const TVector& b) { return TFloat::dotProduct(a, b); }
	
	Vector getNormal() const { return TVector(TFloat::getNormal()); }
	Vector& normalize() { TFloat::normalize(); return *this; }
	
	using TFloat::length;
	using TFloat::length2;
	static bool isNearlyEqual(const TVector& a, const TVector& b) { return TFloat::isNearlyEqual(a, b); }

	friend struct Vector3;
	friend struct Vector4;
	friend struct Quaternion;
};

template <int N>
inline Vector<N> operator*(float lhs, const Vector<N>& rhs)
{
	return rhs * lhs;
}

#endif