#ifndef OC_QUATERNION
#define OC_QUATERNION

#include "NFloat.h"
#include "Vector3.h"

struct Quaternion : private NFloat<4>
{
	NFloat<3>& axis = *((NFloat<3>*)(&data[0]));
	float& w = NFloat<4>::data[3];

	Quaternion()
	{
	}

	Quaternion(const Vector3& inAxis, float inAngle)
	{
		float halfHangle = inAngle * 0.5f;
		float sinHalfAngle = std::sin(halfHangle);
		axis = inAxis.toNFloat() * sinHalfAngle;
		w = std::cos(halfHangle);
	}

	Quaternion(float x, float y, float z, float inW)
	{
		axis[0] = x;
		axis[1] = y;
		axis[2] = z;
		w = inW;
	}

	explicit Quaternion(const NFloat<4>& rhs)
		: NFloat<4>(rhs)
	{
	}

	Quaternion& operator=(const Quaternion& rhs){ NFloat<4>::operator=(rhs); return *this; }

	NFloat<4> toNFloat() const
	{
		return NFloat<4>(axis[0], axis[1], axis[2], w);
	}

	void toAxisAngle(Vector3& outAxis, float& angle) const
	{
		angle = 2.f * std::acos(w);
		float oneOverDenom = 1.f / std::sqrt(1.f - w*w);
		outAxis = Vector3(axis) * oneOverDenom;
	}

	Vector3 operator*(const Vector3& v) const
	{
		//http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
		Vector3 t = Vector3::crossProduct(Vector3(axis), v) * 2.f;
		return v + (t * w) + Vector3::crossProduct(Vector3(axis), t);
	}

	Quaternion operator*(const Quaternion& rhs) const
	{
		const NFloat<4>& a = toNFloat();
		const NFloat<4>& b = rhs.toNFloat();
		NFloat<4> result;

		const float tx = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
		const float ty = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
		const float tz = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
		const float tw = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];

		return Quaternion(tx, ty, tz, tw);
	}

	Quaternion getInverse() const
	{
		return Quaternion(-axis[0], -axis[1], -axis[2], w);
	}

	Quaternion getNormal() const { return Quaternion(NFloat<4>::getNormal()); }
	Quaternion& normalize() { NFloat<4>::normalize(); return *this; }

	using NFloat<4>::length;
	using NFloat<4>::length2;
	static bool isNearlyEqual(const Quaternion& a, const Quaternion& b) { return NFloat<4>::isNearlyEqual(a, b); }
};

#endif