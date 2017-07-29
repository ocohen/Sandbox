#ifndef OC_PAIRWISEOPS_H
#define OC_PAIRWISEOPS_H

#include "VectorOps.h"

template <typename T, typename Scaler>
struct PairwiseOps : public VectorOps<T, Scaler>
{
    typedef VectorOps<T,Scaler> VOps;
    T operator+(Scaler rhs) const{ return VOps::pairwise(rhs, std::plus<Scaler>()); }
    T operator-(Scaler rhs) const{ return VOps::pairwise(rhs, std::minus<Scaler>()); }
    T operator*(const T& rhs) const{ return VOps::pairwise(rhs, std::multiplies<Scaler>()); }

    T& operator+=(Scaler rhs)
    {
        T& ret = VOps::get();
        ret = ret + rhs;
        return ret;
    }

    T& operator-=(Scaler rhs)
    {
        T& ret = VOps::get();
        ret = ret - rhs;
        return ret;
    }

    T& operator*=(const T&rhs)
    {
        T& ret = VOps::get();
        ret = ret * rhs;
        return ret;
    }
};

template <typename T, typename Scaler> T operator+(Scaler lhs, const PairwiseOps<T,Scaler>& rhs){ return rhs + lhs; }
template <typename T, typename Scaler> T operator-(Scaler lhs, const PairwiseOps<T,Scaler>& rhs){ return rhs - lhs; }

#endif
