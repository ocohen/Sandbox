#ifndef OC_RIGIDBODY_H
#define OC_RIGIDBODY_H

#include <vector>
#include "Vector3.h"
#include "Transform.h"
#include "Shape.h"
#include "MassProperties.h"

struct RigidBodyDesc
{
    RigidBodyDesc()
        : com(0.f)
        , invInertia(1.f, 1.f, 1.f)
        , invMass(1.f)
        , linearDamping(0.f)
        , angularDamping(0.f)
    {
    }

    void finalize()
    {
        MassProperties props;
        for (const ShapeUnion& shapeUnion : shapes)
        {
            props.addShape(shapeUnion, 1.f);
        }

        com = props.com;
        invMass = props.mass == 0.f ? 0.f : 1.f / props.mass;
        invInertia = invInertia.x != 0.f || invInertia.y != 0.f || invInertia.z != 0.f ? Vector3(1.f / props.inertia) : invInertia;
    }

    Vector3 com;
    Vector3 invInertia;
    float invMass;
    float linearDamping;
    float angularDamping;
    std::vector<ShapeUnion> shapes;
};



struct RigidBody
{
    RigidBody(const Transform& actorToWorld, const RigidBodyDesc& rigidBodyDesc)
        : invInertia(rigidBodyDesc.invInertia)
        , invMass(rigidBodyDesc.invMass)
        , linearDamping(rigidBodyDesc.linearDamping)
        , angularDamping(rigidBodyDesc.angularDamping)
        , linearVelocity(0.f, 0.f, 0.f)
        , angularVelocity(0.f, 0.f, 0.f)
        , shapes(rigidBodyDesc.shapes)
    {
        const Transform localTM(rigidBodyDesc.com, Quaternion(0.f, 0.f, 0.f, 1.f)); //TODO: compute inertia tensor
        bodyToWorld = actorToWorld * localTM;

        //need to update shape transforms to be relative to the new body transform. We can use the localTM for this
        for (ShapeUnion& shapeUnion : shapes)
        {
            Transform& shapeTM = shapeUnion.asShape().localTM;
            shapeTM = localTM.inverseTransform(shapeTM);
        }
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
