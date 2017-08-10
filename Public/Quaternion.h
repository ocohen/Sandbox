#ifndef OC_QUATERNION
#define OC_QUATERNION

#include "VectorOps.h"
#include "TVector3.h"

template <typename Scaler>
struct TQuaternion : private VectorOps<TQuaternion<Scaler>, Scaler>
{
    enum {order = 4};
    TVector3<Scaler> imaginary;
    Scaler real;

    TQuaternion(){}

    TQuaternion(Scaler x, Scaler y, Scaler z, Scaler w) : imaginary(x,y,z), real(w){}
    TQuaternion(const TVector3<Scaler>& inImaginary, Scaler inReal) : imaginary(inImaginary), real(inReal){}    //NOTE: this is _not_ axis and angle, see fromAxisAndAngle
    TQuaternion(const Scaler3<Scaler>& inImaginary, Scaler inReal) : imaginary(inImaginary), real(inReal){}    //NOTE: this is _not_ axis and angle, see fromAxisAndAngle

    Scaler operator[](int index) const { const Scaler* vals = &imaginary.x; return vals[index]; }
    Scaler& operator[](int index) { Scaler* vals = &imaginary.x; return vals[index]; }

    static TQuaternion fromAxisAndAngle(const TVector3<Scaler>& axis, Scaler angle)
    {
        const Scaler halfHangle = angle * static_cast<Scaler>(0.5);
        const Scaler sinHalfAngle = std::sin(halfHangle);
        const Scaler cosHalfAngle = std::cos(halfHangle);
        return TQuaternion(axis.getSafeNormal() * sinHalfAngle, cosHalfAngle);
    }

    void toAxisAngle(TVector3<Scaler>& outAxis, Scaler& angle) const
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

    TVector3<Scaler> rotateVector(const TVector3<Scaler>& R) const
    {
        //http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
        const TVector3<Scaler>& Q = imaginary;
        const Scaler q = real;
        return  R + static_cast<Scaler>(2.0)*q*TVector3<Scaler>::crossProduct(Q,R) + TVector3<Scaler>::crossProduct(static_cast<Scaler>(2.0)*Q, TVector3<Scaler>::crossProduct(Q,R));
    }

    TVector3<Scaler> operator*(const TVector3<Scaler>& R) const
    {
        return rotateVector(R);
    }

    TQuaternion operator*(const TQuaternion& rhs) const
    {
        //http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
        const TVector3<Scaler>& P = imaginary;
        const TVector3<Scaler>& Q = rhs.imaginary;
        const Scaler p = real;
        const Scaler q = rhs.real;
        return TQuaternion((p*Q + q*P + TVector3<Scaler>::crossProduct(P,Q)), p*q - TVector3<Scaler>::dotProduct(P,Q));
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
    using VOps::isNearlyEqual;

    friend VOps;
};

typedef TQuaternion<float> Quaternion;
typedef TQuaternion<double> QuaternionD;

#endif
