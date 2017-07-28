#ifndef OC_VECTOROPS_H
#define OC_VECTOROPS_H

#include <functional>

template <typename T> struct FloatOps;

template <typename T>
struct VectorOps
{
    T& get() {return *static_cast<T*>(this); }
    const T& get() const {return *static_cast<const T*>(this); }

    FloatOps<T>& asFloat(){ return *static_cast<FloatOps<T>*>(this); }
    const FloatOps<T>& asFloat() const { return *static_cast<const FloatOps<T>*>(this); }

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
    T pairwise(float rhs, Func func) const
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
    T& pairwise(float rhs, Func func)
    {
        T& ret = get();
        ret = pairwise(rhs, func);
        return ret;
    }

    static float dotProduct(const T& lhs, const T& rhs)
    {
        float sum = 0.f;
        for(int i=0; i<T::order; ++i)
        {
            sum += lhs[i] * rhs[i];
        }
        return sum;
    }

    float dotProduct(const T& rhs) const
    {
        const T& lhs = get();
        return dotProduct(lhs, rhs);
    }

    float length2() const
    {
        return dotProduct(get());
    }

    float length() const
    {
        return std::sqrt(length2());
    }

    T getNormal() const
    {
        const float oneOver = 1.f / length();
        return get() * oneOver;
    }

    T& normalize()
    {
        T& ret = get();
        ret = getNormal();
        return ret;
    }

    T operator+(const T& rhs) const{ return pairwise(rhs, std::plus<float>()); }
    T operator-(const T& rhs) const{ return pairwise(rhs, std::minus<float>()); }
    T operator*(float rhs) const{ return pairwise(rhs, std::multiplies<float>()); }
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

    T& operator*=(float rhs)
    {
        T& ret = get();
        ret = ret * rhs;
        return ret;
    }
};

template <typename T> T operator*(float lhs, const VectorOps<T>& rhs){ return rhs * lhs; }

#endif
