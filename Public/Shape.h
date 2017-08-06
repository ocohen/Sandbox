#ifndef OC_SHAPE_H
#define OC_SHAPE_H

#include <cassert>
#include "Renderer.h"
#include "Transform.h"
#include "Vector3.h"

namespace EShape
{
    enum Type
    {
        Sphere,
        Box
    };
}

struct Shape
{
    Transform localTM;
    Vector3 invInertia;
    float invMass;
    EShape::Type type;

    Shape(const Transform& inLocalTM, EShape::Type inType)
        : localTM(inLocalTM)
        , invInertia(Vector3(1.f))
        , invMass(1.f)
        , type(inType)
    {
    }

    template <typename T> const T& as() const { assert(T::ShapeType == type); return *static_cast<const T*>(this); }
    template <typename T> T& as() { assert(T::ShapeType == type); return *static_cast<T*>(this); }
};

struct Sphere : public Shape
{
    enum {ShapeType = EShape::Sphere};

    Sphere(float inRadius, const Transform& inLocalTM )
        : radius(inRadius)
        , Shape(inLocalTM, EShape::Sphere)
    {
    }

    float radius;
};

struct Box : public Shape
{
    enum {ShapeType = EShape::Box};

    Box(const Vector3& inHalfExtents, const Transform& inLocalTM )
        : halfExtents(inHalfExtents)
        , Shape(inLocalTM, EShape::Box)
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

    const Sphere& asSphere() const { assert(EShape::Sphere == shape.type); return sphere; }
    Sphere& asSphere() { assert(EShape::Sphere == shape.type); return sphere; }

    const Box& asBox() const { assert(EShape::Box == shape.type); return box; }
    Box& asBox() { assert(EShape::Box == shape.type); return box; }
};

inline void renderShape(const Shape& shape, const Transform& poseTM, Renderer& renderer)
{
    const Transform worldTM = poseTM * shape.localTM;
    switch(shape.type)
    {
    case EShape::Sphere: renderer.drawSphere(worldTM.translation, shape.as<Sphere>().radius, 8); break;
    //case EShapeType::Box: renderer.drawBox(worldTM, shape.as<Box>().halfExtents); break;
    default: assert(false);
    }
}

#endif