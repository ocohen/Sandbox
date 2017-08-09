#ifndef OC_RIGIDBODY_H
#define OC_RIGIDBODY_H

#include <vector>
#include "Vector3.h"
#include "Transform.h"
#include "Shape.h"

struct RigidBodyDesc
{
    RigidBodyDesc()
        : invInertia(1.f, 1.f, 1.f)
        , invMass(1.f)
        , linearDamping(0.f)
        , angularDamping(0.f)
    {
    }

    Vector3 invInertia;
    float invMass;
    float linearDamping;
    float angularDamping;
    std::vector<ShapeUnion> shapes;
};

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

struct RigidBody
{
    RigidBody()
    {
    }

    RigidBody(const Transform& inBodyToWorld, const RigidBodyDesc& rigidBodyDesc)
        : invInertia(rigidBodyDesc.invInertia)
        , invMass(rigidBodyDesc.invMass)
        , linearDamping(rigidBodyDesc.linearDamping)
        , angularDamping(rigidBodyDesc.angularDamping)
        , linearVelocity(0.f, 0.f, 0.f)
        , angularVelocity(0.f, 0.f, 0.f)
        , bodyToWorld(inBodyToWorld)
        , shapes(rigidBodyDesc.shapes)
    {
    }

    Vector3 invInertia;
	float invMass;
    float linearDamping;
    float angularDamping;

	Vector3 linearVelocity;
	Vector3 angularVelocity;
	Transform bodyToWorld;

    std::vector<ShapeUnion> shapes;
};

#endif
