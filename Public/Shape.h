#ifndef OC_SHAPE_H
#define OC_SHAPE_H

#include <cassert>
#include "Transform.h"
#include "Vector3.h"

enum class EShapeType
{
    Sphere,
    Box
};

struct Shape
{
    Transform localTM;
    Vector3 invInertia;
    float invMass;
    EShapeType type;

    Shape(const Transform& inLocalTM, EShapeType inType)
        : localTM(inLocalTM)
        , invInertia(Vector3(1.f))
        , invMass(1.f)
        , type(inType)
    {
    }

    template <typename T> const T& as() const { assert(T::shapeType == type); return *static_cast<const T*>(this); }
    template <typename T> T& as() { assert(T::shapeType == type); return *static_cast<T*>(this); }
};

struct Sphere : public Shape
{
    static const EShapeType shapeType = EShapeType::Sphere;

    Sphere(float inRadius, const Transform& inLocalTM )
        : radius(inRadius)
        , Shape(inLocalTM, shapeType)
    {
    }

    float radius;
};

struct Box : public Shape
{
    static const EShapeType shapeType = EShapeType::Box;

    Box(const Vector3& inHalfExtents, const Transform& inLocalTM )
        : halfExtents(inHalfExtents)
        , Shape(inLocalTM, shapeType)
    {
    }

    Vector3 halfExtents;
};

union ShapeUnion
{
    Shape shape;
    Sphere sphere;
    Box box;

    ShapeUnion(Sphere inSphere) : sphere(inSphere){}
    ShapeUnion(Box inBox) : box(inBox){}

    const Shape& asShape() const { return shape; }
    Shape& asShape() { return shape; }

    const Sphere& asSphere() const { assert(EShapeType::Sphere == shape.type); return sphere; }
    Sphere& asSphere() { assert(EShapeType::Sphere == shape.type); return sphere; }

    const Box& asBox() const { assert(EShapeType::Box == shape.type); return box; }
    Box& asBox() { assert(EShapeType::Box == shape.type); return box; }
};


#endif
