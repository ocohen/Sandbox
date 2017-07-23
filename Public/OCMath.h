#ifndef OC_MATH
#define OC_MATH

//#include <cmath>

#define OC_EPSILON 1e-8
#define OC_BIG_EPSILON 1e-4
#define PI 3.14159265359f
#define PI_OVER_TWO 1.57079632679f

inline bool isNearlyEqual(float x, float y, float tolerance = OC_BIG_EPSILON)
{
	const float diff = y-x;
	return (diff >= -tolerance) && (diff <= tolerance);
}

#endif