#ifndef OC_EQUALOPS_H
#define OC_EQUALOPS_H

#include "OCMath.h"

template <typename T>
struct EqualOps
{
    static bool isNearlyEqual(const T& lhs, const T& rhs)
    {
        bool isEqual = true;
        for(int i =0; i< T::order && isEqual; ++i)
        {
            isEqual = ::isNearlyEqual(lhs[i], rhs[i]);
        }
        return isEqual;
    }
};

#endif
