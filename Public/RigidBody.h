#ifndef OC_RIGIDBODY_H
#define OC_RIGIDBODY_H

#include "Vector3.h"
#include "Transform.h"

struct RigidBody
{
	float invMass;
	Vector3 invInertia;

	Vector3 linearVelocity;
	Vector3 angularVelocity;
	Transform bodyToWorld;
};

#endif