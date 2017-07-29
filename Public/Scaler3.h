#ifndef OC_SCALER3
#define OC_SCALER3

#include "PairwiseOps.h"
#include "EqualOps.h"

template <typename Scaler>
struct Scaler3 : public PairwiseOps<Scaler3<Scaler>, Scaler>, public EqualOps<Scaler3<Scaler>>
{
    enum{ order=3 };

    Scaler x;
    Scaler y;
    Scaler z;

    Scaler3(){}
    Scaler3(Scaler inX, Scaler inY, Scaler inZ): x(inX), y(inY), z(inZ){}
    Scaler3(const Scaler3& rhs) : x(rhs.x), y(rhs.y), z(rhs.z){}
    explicit Scaler3(Scaler k) : x(k), y(k), z(k) {}

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
