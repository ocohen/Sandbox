#ifndef OC_QUATERNION
#define OC_QUATERNION

#include "VectorOps.h"
#include "Vector3.h"

struct Quaternion : private VectorOps<Quaternion>, public EqualOps<Quaternion>
{
    Vector3 imaginary;
    float real;

	Quaternion(){}

	Quaternion(float x, float y, float z, float w) : imaginary(x,y,z), real(w){}
	Quaternion(const Vector3& inImaginary, float inReal) : imaginary(inImaginary), real(inReal){}	//NOTE: this is _not_ axis and angle, see fromAxisAndAngle
	Quaternion(const Float3& inImaginary, float inReal) : imaginary(inImaginary), real(inReal){}	//NOTE: this is _not_ axis and angle, see fromAxisAndAngle

	Quaternion(const Quaternion& other) : imaginary(other.imaginary), real(other.real){}

	Quaternion& operator=(const Quaternion& rhs)
	{
        imaginary = rhs.imaginary;
        real = rhs.real;
		return *this;
	}

    float operator[](int index) const { const float* vals = &imaginary.x; return vals[index]; }
    float& operator[](int index) { float* vals = &imaginary.x; return vals[index]; }

	static Quaternion fromAxisAndAngle(const Vector3& axis, float angle)
	{
		const float halfHangle = angle * 0.5f;
		const float sinHalfAngle = std::sin(halfHangle);
		const float cosHalfAngle = std::cos(halfHangle);
		return Quaternion(axis * sinHalfAngle, cosHalfAngle);
	}

	void toAxisAngle(Vector3& outAxis, float& angle) const
	{
		angle = 2.f * std::acos(real);
		const float denom = 1.f - real*real;
		if(denom > OC_BIG_EPSILON)
		{
			const float oneOverDenom = 1.f / std::sqrt(denom);
			outAxis = imaginary * oneOverDenom;
		}
		else
		{
			outAxis = imaginary;
		}
	}

	Vector3 rotateVector(const Vector3& R) const
	{
		//http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
		const Vector3& Q = imaginary;
		const float q = real;
		return  R + 2.f*q*Vector3::crossProduct(Q,R) + Vector3::crossProduct(2.f*Q, Vector3::crossProduct(Q,R));
	}

	Vector3 operator*(const Vector3& R) const
	{
		return rotateVector(R);
	}

	Quaternion operator*(const Quaternion& rhs) const
	{
		//http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
		const Vector3& P = imaginary;
		const Vector3& Q = rhs.imaginary;
		const float p = real;
		const float q = rhs.real;
		return Quaternion((p*Q + q*P + Vector3::crossProduct(P,Q)), p*q - Vector3::dotProduct(P,Q));
	}

	Quaternion getInverse() const
	{
		return Quaternion(-imaginary, real);
	}

    using VectorOps<Quaternion>::getNormal;
    using VectorOps<Quaternion>::normalize;
    using VectorOps<Quaternion>::length;
    using VectorOps<Quaternion>::length2;
};

#endif
