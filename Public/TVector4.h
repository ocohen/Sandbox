#ifndef OC_TVECTOR4
#define OC_TVECTOR4

#include "VectorOps.h"
#include "EqualOps.h"
#include "Scaler4.h"

template <typename Scaler>
struct TVector4 : public VectorOps<TVector4<Scaler>, Scaler>
{
    enum{ order=4 };
    Scaler x;
    Scaler y;
    Scaler z;
    Scaler w;

    TVector4(){}
    TVector4(Scaler inX, Scaler inY, Scaler inZ, Scaler inW): x(inX), y(inY), z(inZ), w(inW){}
    explicit TVector4(Scaler k) : x(k), y(k), z(k), w(k) {}
    explicit TVector4(const Scaler4<Scaler>& other) : x(other.x), y(other.y), z(other.z), w(other.w){}

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
};
#endif
