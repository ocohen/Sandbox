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

Vector3 getClosestPointOnTriangle(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& p)
{
    //See orange collision detection book
    //compute barycentric coordinates of the projection of p onto the triangle abc
    const Vector3 ab = b-a;
    const Vector3 ac = c-a;
    const Vector3 bc = c-b;

    const Vector3 ap = p-a;
    const Vector3 bp = p-b;
    const Vector3 cp = p-c;

    //find p' = A + s*AB which is the projection of p onto ab
    const float sNom = Vector3::dotProduct(ap, ab);
    const float sDenom = Vector3::dotProduct(bp, a - b);

    //find p' = A + t*AC which is the projection of p onto ac
    const float tNom = Vector3::dotProduct(ap, ac);
    const float tDenom = Vector3::dotProduct(cp, a - c);

    if(sNom <= 0.f && tNom <= 0.f)
    {
        return a;
    }

    //find p' = B + u*BC which is the projection of p onto bc
    const float uNom = Vector3::dotProduct(bp, bc);
    const float uDenom = Vector3::dotProduct(cp, b - c);

    if(sDenom <= 0.f && uNom <= 0.f)
    {
        return b;
    }

    if(tDenom <= 0.f && uDenom <= 0.f)
    {
        return c;
    }

    const Vector3 n = Vector3::crossProduct(ab, ac);
    const float vC = Vector3::dotProduct(n, Vector3::crossProduct(a - p, b - p));

    if(vC <= 0.f && sNom >= 0.f && sDenom >= 0.f)
    {
        return a + sNom / (sNom + sDenom) * ab;
    }

    const float vA = Vector3::dotProduct(n, Vector3::crossProduct(b - p, c - p));
    if (vA <= 0.f && uNom >= 0.f && uDenom >= 0.f)
    {
        return b + uNom / (uNom + uDenom) * bc;
    }

    const float vB = Vector3::dotProduct(n, Vector3::crossProduct(c - p, a - p));
    if (vB <= 0.f && tNom >= 0.f && tDenom >= 0.f)
    {
        return a + tNom / (tNom + tDenom) * ac;
    }

    //P projects inside the triangle face
    const float u = vA / (vA + vB + vC);
    const float v = vB / (vA + vB + vC);
    const float w = 1.f - u - v;
    return u*a + v*b + w*c;
}

#endif
