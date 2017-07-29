#ifndef OC_TVECTOR3
#define OC_TVECTOR3

#include "VectorOps.h"
#include "EqualOps.h"
#include "Scaler3.h"

template <typename Scaler>
struct TVector3 : public VectorOps<TVector3<Scaler>, Scaler>, public EqualOps<TVector3<Scaler>>
{
    enum{ order=3 };
    Scaler x;
    Scaler y;
    Scaler z;

    TVector3(){}
    TVector3(Scaler inX, Scaler inY, Scaler inZ): x(inX), y(inY), z(inZ){}
    explicit TVector3(Scaler k) : x(k), y(k), z(k) {}
    explicit TVector3(const Scaler3<Scaler>& other) : x(other.x), y(other.y), z(other.z){}

    Scaler operator[](int index) const
    {
       const Scaler* vals = &x;
       return vals[index]; 
    }

    Scaler& operator[](int index)
    {
       Scaler* vals = &x;
       return vals[index]; 
    }

    static TVector3 crossProduct(const TVector3& a, const TVector3& b)
    {
        return TVector3(a.y*b.z - a.z*b.y, -(a.x*b.z - a.z*b.x), a.x*b.y - a.y*b.x);
    }
};
#endif
