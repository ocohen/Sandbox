#ifndef OC_SHAPERENDER_H
#define OC_SHAPERENDER_H

#include "Shape.h"
#include "Renderer.h"

inline void renderShape(const Shape& shape, const Transform& poseTM, Renderer& renderer, Vector3* color = nullptr)
{
    const Transform worldTM = poseTM * shape.localTM;
    switch(shape.type)
    {
    case EShapeType::Sphere: renderer.drawSphere(worldTM.translation, shape.as<Sphere>().radius, 8, color); break;
    case EShapeType::Box: renderer.drawBox(worldTM, shape.as<Box>().halfExtents, color); break;
    default: assert(false);
    }
}

#endif
