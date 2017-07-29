#ifndef OC_SCALER4
#define OC_SCALER4

#include "PairwiseOps.h"
#include "EqualOps.h"

template <typename Scaler>
struct Scaler4 : public PairwiseOps<Scaler4<Scaler>,Scaler>, public EqualOps<Scaler4<Scaler>>
{
    enum{ order=4 };

    Scaler x;
    Scaler y;
    Scaler z;
    Scaler w;

    Scaler4(){}
    Scaler4(Scaler inX, Scaler inY, Scaler inZ, Scaler inW): x(inX), y(inY), z(inZ), w(inW){}
    explicit Scaler4(Scaler k) : x(k), y(k), z(k), w(k){}

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
