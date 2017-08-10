#ifndef OC_MASSPROPERTIES_H
#define OC_MASSPROPERTIES_H

#include "Shape.h"
#include "Vector3.h"
#include "Transform.h"

inline float computeVolume(const ShapeUnion& shapeUnion)
{
    switch(shapeUnion.shape.type)
    {
        case EShapeType::Sphere:
        {
            const float radius = shapeUnion.asSphere().radius;
            return (4.f / 3.f) * radius * radius * radius * PI;
        }
        case EShapeType::Box:
        {
            const Box& box = shapeUnion.asBox();
            return box.halfExtents.x * box.halfExtents.y * box.halfExtents.z * 8.f;
        }
        default: assert(false); return 0.f;
    }
}

struct MassProperties
{
    //Matrix33 inertia;
    Vector3 com;
    float mass;

    MassProperties()
    : com(0.f, 0.f, 0.f)
    , mass(0.f)
    {
    }

    void addShape(const ShapeUnion& shapeUnion, float density)
    {
        const float vol = computeVolume(shapeUnion);
        const float shapeMass = vol * density; 
        float newTotalMass = mass + shapeMass;
        float comWeight = shapeMass / newTotalMass;
        com = shapeUnion.shape.localTM.translation * comWeight + (1.f - comWeight) * com;
        mass = newTotalMass;
        //todo inertia
    }
};

#endif
