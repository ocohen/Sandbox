#ifndef OC_GJK
#define OC_GJK

#include "Vector3.h"

Vector3 getClosestPointOnLineSegment(const Vector3& lineStart, const Vector3& lineEnd, const Vector3& pt)
{
    //assume line from a to b and pt is c projected onto it. Clamp t from 0-1
    Vector3 ab = lineEnd - lineStart;
    const float abLength2 = ab.length2();
    if(isNearlyEqual(abLength2, 0.f))
    {
        return lineStart;
    }

    Vector3 ac = pt - lineStart;
    float t = Vector3::dotProduct(ac, ab) / abLength2;
    if(t <= 0.f) return lineStart;
    if(t >= 1.f) return lineEnd;
    return lineStart + t * ab;
}

#endif
