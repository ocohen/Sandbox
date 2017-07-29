#ifndef OC_QUATERNION
#define OC_QUATERNION

#include "VectorOps.h"
#include "Vector3.h"

template <typename Scaler>
struct TQuaternion : private VectorOps<TQuaternion<Scaler>, Scaler>, public EqualOps<TQuaternion<Scaler>>
{
    Vector3 imaginary;
    Scaler real;

    TQuaternion(){}

    TQuaternion(Scaler x, Scaler y, Scaler z, Scaler w) : imaginary(x,y,z), real(w){}
    TQuaternion(const Vector3& inImaginary, Scaler inReal) : imaginary(inImaginary), real(inReal){}    //NOTE: this is _not_ axis and angle, see fromAxisAndAngle
    TQuaternion(const Scaler3<Scaler>& inImaginary, Scaler inReal) : imaginary(inImaginary), real(inReal){}    //NOTE: this is _not_ axis and angle, see fromAxisAndAngle

    Scaler operator[](int index) const { const Scaler* vals = &imaginary.x; return vals[index]; }
    Scaler& operator[](int index) { Scaler* vals = &imaginary.x; return vals[index]; }

    static TQuaternion fromAxisAndAngle(const Vector3& axis, Scaler angle)
    {
        const Scaler halfHangle = angle * static_cast<Scaler>(0.5);
        const Scaler sinHalfAngle = std::sin(halfHangle);
        const Scaler cosHalfAngle = std::cos(halfHangle);
        return TQuaternion(axis * sinHalfAngle, cosHalfAngle);
    }

    void toAxisAngle(Vector3& outAxis, Scaler& angle) const
    {
        angle = static_cast<Scaler>(2.0) * std::acos(real);
        const Scaler denom = static_cast<Scaler>(1.0) - real*real;
        if(denom > OC_BIG_EPSILON)
        {
            const Scaler oneOverDenom = static_cast<Scaler>(1.0) / std::sqrt(denom);
            outAxis = imaginary * oneOverDenom;
        }
        else
        {
            outAxis = imaginary;
        }
    }

    Vector3 rotateVector(const Vector3& R) const
    {
        //http://people.csail.mit.edu/bkph/articles/TQuaternions.pdf
        const Vector3& Q = imaginary;
        const Scaler q = real;
        return  R + static_cast<Scaler>(2.0)*q*Vector3::crossProduct(Q,R) + Vector3::crossProduct(static_cast<Scaler>(2.0)*Q, Vector3::crossProduct(Q,R));
    }

    Vector3 operator*(const Vector3& R) const
    {
        return rotateVector(R);
    }

    TQuaternion operator*(const TQuaternion& rhs) const
    {
        //http://people.csail.mit.edu/bkph/articles/TQuaternions.pdf
        const Vector3& P = imaginary;
        const Vector3& Q = rhs.imaginary;
        const Scaler p = real;
        const Scaler q = rhs.real;
        return TQuaternion((p*Q + q*P + Vector3::crossProduct(P,Q)), p*q - Vector3::dotProduct(P,Q));
    }

    TQuaternion getInverse() const
    {
        return TQuaternion(-imaginary, real);
    }

    typedef VectorOps<TQuaternion,Scaler> VOps;

    using VOps::getNormal;
    using VOps::normalize;
    using VOps::length;
    using VOps::length2;
};

typedef TQuaternion<float> Quaternion;
typedef TQuaternion<double> QuaternionD;

#endif
