#ifndef OC_VECTOROPS_H
#define OC_VECTOROPS_H

#include <functional>
#include "EqualOps.h"

template <typename T, typename Scaler> struct PairwiseOps;

template <typename T, typename Scaler>
struct VectorOps : public EqualOps<T>
{
    T& get() {return *static_cast<T*>(this); }
    const T& get() const {return *static_cast<const T*>(this); }

    PairwiseOps<T, Scaler>& asScalers(){ return *static_cast<PairwiseOps<T, Scaler>*>(this); }
    const PairwiseOps<T, Scaler>& asScalers() const { return *static_cast<const PairwiseOps<T, Scaler>*>(this); }

    template<typename Func>
    T pairwise(const T& rhs, Func func) const
    {
        T ret;
        const T& lhs = get();
        for(int i=0; i<T::order; ++i)
        {
            ret[i] = func(lhs[i],rhs[i]);
        }
        return ret;
    }


    template<typename Func>
    T& pairwise(const T& rhs, Func func)
    {
        T& ret = get();
        ret = pairwise(rhs, func);
        return ret;
    }

    template<typename Func>
    T pairwise(Scaler rhs, Func func) const
    {
        T ret;
        const T& lhs = get();
        for(int i=0; i<T::order; ++i)
        {
            ret[i] = func(lhs[i],rhs);
        }
        return ret;
    }


    template<typename Func>
    T& pairwise(Scaler rhs, Func func)
    {
        T& ret = get();
        ret = pairwise(rhs, func);
        return ret;
    }

    static Scaler dotProduct(const T& lhs, const T& rhs)
    {
        Scaler sum = 0.f;
        for(int i=0; i<T::order; ++i)
        {
            sum += lhs[i] * rhs[i];
        }
        return sum;
    }

    Scaler dotProduct(const T& rhs) const
    {
        const T& lhs = get();
        return dotProduct(lhs, rhs);
    }

    Scaler length2() const
    {
        return dotProduct(get());
    }

    Scaler length() const
    {
        return std::sqrt(length2());
    }

    T getNormal() const
    {
        const Scaler oneOver = 1.f / length();
        return get() * oneOver;
    }

    T& normalize()
    {
        T& ret = get();
        ret = getNormal();
        return ret;
    }

    T operator+(const T& rhs) const{ return pairwise(rhs, std::plus<Scaler>()); }
    T operator-(const T& rhs) const{ return pairwise(rhs, std::minus<Scaler>()); }
    T operator*(Scaler rhs) const{ return pairwise(rhs, std::multiplies<Scaler>()); }
    T operator-() const
    {
        T ret;
        const T& t = get();
        for(int i=0; i<T::order; ++i)
        {
            ret[i] = -t[i];
        }
        return ret;
    }


    T& operator+=(const T&rhs)
    {
        T& ret = get();
        ret = ret + rhs;
        return ret;
    }

    T& operator-=(const T&rhs)
    {
        T& ret = get();
        ret = ret - rhs;
        return ret;
    }

    T& operator*=(Scaler rhs)
    {
        T& ret = get();
        ret = ret * rhs;
        return ret;
    }
};

template <typename T, typename Scaler> T operator*(Scaler lhs, const VectorOps<T, Scaler>& rhs){ return rhs * lhs; }
#endif
