#ifndef OC_MATH
#define OC_MATH

//#include <cmath>

#define OC_EPSILON 1e-8

inline bool isNearlyEqual(float x, float y)
{
	const float diff = y-x;
	return (diff > -OC_EPSILON) && (diff < OC_EPSILON);
}

#endif