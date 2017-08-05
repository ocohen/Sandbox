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
    std::vector<Shape*> shapes;
};

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
    {
        for(Shape* shape : rigidBodyDesc.shapes)
        {
            shapes.push_back(shape->clone());
        }
    }

    ~RigidBody()
    {
        for(Shape* shape : shapes)
        {
            delete shape;
        }
    }

    Vector3 invInertia;
	float invMass;
    float linearDamping;
    float angularDamping;

	Vector3 linearVelocity;
	Vector3 angularVelocity;
	Transform bodyToWorld;

    std::vector<Shape*> shapes;   //For now just shove everything into the rigid body instead of breaking it into proper SOA
};

#endif