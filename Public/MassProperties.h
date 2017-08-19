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
    float inertia;  //todo: hack for now to get a simple scaling factor
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
        //total hack for inertia scaling, ignores multiple shapes and 3d
        switch(shapeUnion.shape.type)
        {
        case EShapeType::Sphere: inertia = 2.f*mass*shapeUnion.asSphere().radius*shapeUnion.asSphere().radius / 5.f; break;
        case EShapeType::Box: inertia = mass*shapeUnion.asBox().halfExtents.x*shapeUnion.asBox().halfExtents.x*4.f/6.f; break;  //terrible assumes cube
        default: inertia = 1.f;
        }
    }
};

#endif
