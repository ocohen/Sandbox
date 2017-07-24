#ifndef OC_QUATERNION
#define OC_QUATERNION

#include "Vector3.h"

struct Quaternion : private NFloat<4>
{
	Quaternion()
	{
	}

	Quaternion(float inX, float inY, float inZ, float inW)
	: TFloat {inX, inY, inZ, inW}
	{
	}

	Quaternion(const NFloat<3>& inImaginary, float inReal)	//NOTE: this is _not_ axis and angle, see fromAxisAndAngle
		: TFloat {inImaginary[0], inImaginary[1], inImaginary[2], inReal}
	{
	}

	Quaternion(const Quaternion& other)
	: TFloat {other[0], other[1], other[2], other[3]}
	{
	}

	explicit Quaternion(const TFloat& rhs)
		: TFloat(rhs)
	{
	}

	Quaternion& operator=(const Quaternion& rhs)
	{
		TFloat::operator=(rhs);
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

	static Quaternion fromAxisAndAngle(const Vector3& axis, float angle)
	{
		const float halfHangle = angle * 0.5f;
		const float sinHalfAngle = std::sin(halfHangle);
		const float cosHalfAngle = std::cos(halfHangle);
		return Quaternion(axis.asNFloat() * sinHalfAngle, cosHalfAngle);
	}

	const TFloat& asNFloat() const
	{
		return reinterpret_cast<const TFloat&>(*this);
	}

	TFloat& asNFloat()
	{
		return reinterpret_cast<TFloat&>(*this);
	}

	const Vector3& imaginary() const
	{
		return *reinterpret_cast<const Vector3*>(this);
	}

	Vector3& imaginary()
	{
		return *reinterpret_cast<Vector3*>(this);
	}

	void toAxisAngle(Vector3& outAxis, float& angle) const
	{
		angle = 2.f * std::acos(w());
		const float denom = 1.f - w()*w();
		if(denom > OC_BIG_EPSILON)
		{
			const float oneOverDenom = 1.f / std::sqrt(denom);
			outAxis = imaginary() * oneOverDenom;
		}
		else
		{
			outAxis = imaginary();
		}
	}

	Vector3 rotateVector(const Vector3& R) const
	{
		//http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
		const Vector3& Q = imaginary();
		const float q = w();
		return  R + 2.f*q*Vector3::crossProduct(Q,R) + Vector3::crossProduct(2.f*Q, Vector3::crossProduct(Q,R));
	}

	Vector3 operator*(const Vector3& R) const
	{
		return rotateVector(R);
	}

	Quaternion operator*(const Quaternion& rhs) const
	{
		//http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
		const Vector3& P = imaginary();
		const Vector3& Q = rhs.imaginary();
		const float p = w();
		const float q = rhs.w();
		return Quaternion((p*Q + q*P + Vector3::crossProduct(P,Q)).asNFloat(), p*q - Vector3::dotProduct(P,Q));
	}

	Quaternion getInverse() const
	{
		return Quaternion(-imaginary().asNFloat(), w());
	}

	Quaternion getNormal() const { return Quaternion(NFloat<4>::getNormal()); }
	Quaternion& normalize() { NFloat<4>::normalize(); return *this; }

	static bool isNearlyEqual(const Quaternion& a, const Quaternion& b) { return NFloat<4>::isNearlyEqual(a, b); }
};

#endif