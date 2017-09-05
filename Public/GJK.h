#ifndef OC_GJK
#define OC_GJK

#include "Vector3.h"
#include "Shape.h"
#include "Renderer.h"

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

/** Determines whether two points are on opposing sides of a face */
bool pointsOnOpposingFaceSides(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& p1, const Vector3& p2)
{
    const Vector3 faceNormal = Vector3::crossProduct(b-a,c-a);
    const float signp1 = Vector3::dotProduct(p1 - a, faceNormal);
    const float signp2 = Vector3::dotProduct(p2 - a, faceNormal);
    return signp1 * signp2 < 0.f;
}

Vector3 getClosestPointOnTetrahedron(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d, const Vector3& p)
{
    //We compute the closest point on each triangle and return the shortest. Points within the tetrahedron are of course the closest
    Vector3 closestPt = p;
    float shortestDist2 = FLT_MAX;

    if(pointsOnOpposingFaceSides(a,b,c,d,p))
    {
        const Vector3 closestABC = getClosestPointOnTriangle(a,b,c,p);
        const float dist2 = (closestABC - p).length2();
        if(dist2 < shortestDist2)
        {
            shortestDist2 = dist2;
            closestPt = closestABC;
        }
    }

    if (pointsOnOpposingFaceSides(a, b, d, c, p))
    {
        const Vector3 closestABD = getClosestPointOnTriangle(a, b, d, p);
        const float dist2 = (closestABD - p).length2();
        if (dist2 < shortestDist2)
        {
            shortestDist2 = dist2;
            closestPt = closestABD;
        }
    }

    if (pointsOnOpposingFaceSides(b, d, c, a, p))
    {
        const Vector3 closestBDC = getClosestPointOnTriangle(b, d, c, p);
        const float dist2 = (closestBDC - p).length2();
        if (dist2 < shortestDist2)
        {
            shortestDist2 = dist2;
            closestPt = closestBDC;
        }
    }

    if (pointsOnOpposingFaceSides(a, d, c, b, p))
    {
        const Vector3 closestADC = getClosestPointOnTriangle(a, d, c, p);
        const float dist2 = (closestADC - p).length2();
        if (dist2 < shortestDist2)
        {
            shortestDist2 = dist2;
            closestPt = closestADC;
        }
    }

    return closestPt;
}

Vector3 support(const Sphere& sphere, const Transform& sphereTM, const Vector3& dir)
{
    return sphereTM.translation + sphere.radius * dir.getNormal();
}

Vector3 support(const Box& box, const Transform& boxTM, const Vector3& dir)
{
    float bestAlong = -FLT_MAX;
    const Vector3 orientedExtents = boxTM.transformVector(box.halfExtents);
    //TODO: move dir into local space instead of checking for same direction
    Vector3 choose = orientedExtents;
    for(int i=0; i<3; ++i)
    {
        if (dir[i] * orientedExtents[i] < 0)
        {
            choose[i] *= -1;
        }
    }
    
    return choose + boxTM.translation;
}

Vector3 support(const ShapeUnion& shape, const Transform& shapeTM, const Vector3& dir)
{
    switch(shape.asShape().type)
    {
    case EShapeType::Box: return support(shape.asBox(),shapeTM, dir);
    case EShapeType::Sphere: return support(shape.asSphere(),shapeTM, dir);
    default: return Vector3(0.f);
    }
}

Vector3 supportAMinusB(const ShapeUnion& a, const ShapeUnion& b, const Transform& bLocalToATM, const Vector3& dir)
{
    return support(a, Transform::identity(), dir) - support(b,bLocalToATM, -dir);
}

Vector3 getClosestToOriginInSimplex(const Vector3* simplex, int dimension)
{
    switch (dimension)
    {
    case 0:
    {
        return simplex[0];
    }
    case 1:
    {
        return getClosestPointOnLineSegment(simplex[0], simplex[1], Vector3(0.f));
    }
    case 2:
    {
        return getClosestPointOnTriangle(simplex[0], simplex[1], simplex[2], Vector3(0.f));
    }
    case 3:
        return getClosestPointOnTetrahedron(simplex[0], simplex[1], simplex[2], simplex[3], Vector3(0.f));
    default:
        assert(false && "Simplex of dimension 4 or bigger");
        return Vector3(0.f);
    }
}

void reduceSimplex(Vector3* simplex, int& dimension, const Vector3& closestPt)
{
    switch(dimension)
    {
    case 0:
    case 1:
        return;
    case 2:
    {
        if(Vector3::isNearlyEqual(closestPt, getClosestPointOnLineSegment(simplex[1], simplex[2], closestPt)))
        {
            simplex[0] = simplex[2];
            dimension = 1;
        }
        else if(Vector3::isNearlyEqual(closestPt, getClosestPointOnLineSegment(simplex[0], simplex[2], closestPt)))
        {
            simplex[1] = simplex[2];
            dimension = 1;
        }
        else if(Vector3::isNearlyEqual(closestPt, getClosestPointOnLineSegment(simplex[0], simplex[1], closestPt)))
        {
            dimension = 1;
        }

        return;
    }
    case 3:
    {
        if(Vector3::isNearlyEqual(closestPt, getClosestPointOnTriangle(simplex[1], simplex[2], simplex[3], closestPt)))
        {
            simplex[0] = simplex[3];
            dimension = 2;
        }
        else if(Vector3::isNearlyEqual(closestPt, getClosestPointOnTriangle(simplex[0], simplex[2], simplex[3], closestPt)))
        {
            simplex[1] = simplex[3];
            dimension = 2;
        }
        else if (Vector3::isNearlyEqual(closestPt, getClosestPointOnTriangle(simplex[0], simplex[1], simplex[3], closestPt)))
        {
            simplex[2] = simplex[3];
            dimension = 2;
        }
        else if (Vector3::isNearlyEqual(closestPt, getClosestPointOnTriangle(simplex[0], simplex[1], simplex[2], closestPt)))
        {
            dimension = 2;
        }

        return;
    }
    default:
        assert(false && "Simplex of dimension 4 or higher not supported");
    }
}

struct GJKDebugPerFrameInfo
{
    int dimension;
    Vector3 simplex[4];
    Vector3 closestPt;
    enum EResult
    {
        Overlap,
        NoOverlap,
        NoTermination
    } result;

    GJKDebugPerFrameInfo(int inDimension, const Vector3* inSimplex, const Vector3& inClosest)
        : dimension(inDimension)
        , closestPt(inClosest)
        , result(NoTermination)
    {
        for(int i=0; i<=inDimension; ++i)
        {
            simplex[i] = inSimplex[i];
        }
    }
};

struct GJKDebugInfo
{
    int numIterations;
    std::vector<GJKDebugPerFrameInfo> perFrameInfo;

    int hullResolution;
    std::vector<Vector3> hullVerts;
};

template <bool debug>
bool gjkOverlappingImp(const ShapeUnion& a, const Transform& a2World, const ShapeUnion& b, const Transform& b2World, GJKDebugInfo* debugInfo)
{
    const Transform bLocal2A = a2World.inverseTransform(b2World);

    if(debug && debugInfo)
    {
        //try sweeping a sphere to generate the hull for debug render
        for(int i=0; i<debugInfo->hullResolution; ++i)
        {
            const float theta = i * 2*PI / debugInfo->hullResolution;
            Vector3 dir(cosf(theta), sinf(theta), 0.f);
            debugInfo->hullVerts.push_back(supportAMinusB(a, b, bLocal2A, dir));
        }
    }

    Vector3 simplex[4];
    simplex[0] = supportAMinusB(a, b, bLocal2A, Vector3(1.f, 0.f, 0.f));
    int dimension = 0;
    float prevDist = FLT_MAX;

    Vector3 vertsSeen[128];
    int numVertsSeen = 0;
    int lastVertSeen = 0;
    int iterationCount = 0;
    bool trackVerts = false;

    while(true)
    {
       if(debug && debugInfo && debugInfo->numIterations <= iterationCount)
       {
           return false;
       }
       ++iterationCount;

       Vector3 closestPt = getClosestToOriginInSimplex(simplex, dimension);
       const bool bContainsOrigin = Vector3::isNearlyEqual(closestPt, Vector3(0.f));

       if(debug && debugInfo)
       {
           debugInfo->perFrameInfo.push_back(GJKDebugPerFrameInfo(dimension, simplex, closestPt));
       }

       if(bContainsOrigin)
       {
           if(debug && debugInfo)
           {
               debugInfo->perFrameInfo.back().result = GJKDebugPerFrameInfo::Overlap;
           }
           return true;
       }
       else
       {
           reduceSimplex(simplex, dimension, closestPt);
       }

       const Vector3 searchDir = -closestPt.getNormal();
       Vector3 newVertex = supportAMinusB(a, b, bLocal2A, searchDir);
       const float progress = (newVertex - closestPt).dotProduct(searchDir);
       const float distFromOrigin = newVertex.length();
       if(progress > 1.f)   //need epsilon for rounded edges where we can make very tiny progress
       {
           if(true || prevDist > distFromOrigin + 0.1f)
           {
               simplex[++dimension] = newVertex;
               prevDist = distFromOrigin;

               if (trackVerts)
               {
                   for(int i=0; i<numVertsSeen; ++i)
                   {
                       if(Vector3::isNearlyEqual(vertsSeen[i], newVertex))
                       {
                           if (debug && debugInfo)
                           {
                               debugInfo->perFrameInfo.back().result = GJKDebugPerFrameInfo::NoOverlap;
                           }
                           return false;
                       }
                   }

                   vertsSeen[lastVertSeen] = newVertex;
                   const size_t sizeOfSeen = (sizeof(vertsSeen) / sizeof(vertsSeen[0]));
                   lastVertSeen = (lastVertSeen + 1) % sizeOfSeen;
                   if(numVertsSeen <sizeOfSeen)
                   {
                       ++numVertsSeen;
                   }
               }
           }
           else
           {
               if (debug && debugInfo)
               {
                   debugInfo->perFrameInfo.back().result = GJKDebugPerFrameInfo::NoOverlap;
               }
                return false;
           }
       }
       else
       {
           if (debug && debugInfo)
           {
               debugInfo->perFrameInfo.back().result = GJKDebugPerFrameInfo::NoOverlap;
           }
           return false;
       }
    }
}

bool gjkOverlapping(const ShapeUnion& a, const Transform& a2World, const ShapeUnion& b, const Transform& b2World)
{
    return gjkOverlappingImp<false>(a, a2World, b, b2World, nullptr);
}

#endif
