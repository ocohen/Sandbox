#ifndef OC_FLOATOPS_H
#define OC_FLOATOPS_H

#include "VectorOps.h"

template <typename T>
struct FloatOps : public VectorOps<T>
{
    T operator+(float rhs) const{ return VectorOps<T>::pairwise(rhs, std::plus<float>()); }
    T operator-(float rhs) const{ return VectorOps<T>::pairwise(rhs, std::minus<float>()); }
    T operator*(const T& rhs) const{ return VectorOps<T>::pairwise(rhs, std::multiplies<float>()); }

    T& operator+=(float rhs)
    {
        T& ret = VectorOps<T>::get();
        ret = ret + rhs;
        return ret;
    }

    T& operator-=(float rhs)
    {
        T& ret = VectorOps<T>::get();
        ret = ret - rhs;
        return ret;
    }

    T& operator*=(const T&rhs)
    {
        T& ret = VectorOps<T>::get();
        ret = ret * rhs;
        return ret;
    }
};

template <typename T> T operator+(float lhs, const FloatOps<T>& rhs){ return rhs + lhs; }
template <typename T> T operator-(float lhs, const FloatOps<T>& rhs){ return rhs - lhs; }

#endif
