#ifndef OC_MATH
#define OC_MATH

#include <cmath>

#define OC_EPSILON 1e-8f
#define OC_BIG_EPSILON 1e-4f
#define OC_EPSILON_DOUBLE 1e-16
#define OC_BIG_EPSILON_DOUBLE 1e-8
#define PI 3.14159265359f
#define PI_OVER_TWO 1.57079632679f

template <typename T>
inline T sign(T x)
{
    return x < (T)0 ? (T)-1 : (x > (T)0 ? (T)1 : (T)0);
}

inline bool isNearlyEqual(float x, float y, float tolerance = OC_BIG_EPSILON)
{
    const float diff = y-x;
    return (diff >= -tolerance) && (diff <= tolerance);
}

inline bool isNearlyEqual(double x, double y, double tolerance = OC_BIG_EPSILON_DOUBLE)
{
    const double diff = y-x;
    return (diff >= -tolerance) && (diff <= tolerance);
}

inline bool isNearlyEqual(int x, int y)
{
    return x == y;
}

inline bool isNearlyZero(float x, float tolerance = OC_BIG_EPSILON)
{
    return isNearlyEqual(x, 0.f);
}

inline float anglesToRadians(float angle)
{
    return PI * angle / 180.f;
}

template <typename T>
T clamp(const T& val, const T& min, const T& max)
{
    return (val < min ? min : (val > max ? max : val));
}

#endif
