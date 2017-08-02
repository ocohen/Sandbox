#ifndef OC_PHYSWORLD_H
#define OC_PHYSWORLD_H

#include <vector>
#include "RigidBody.h"
#include "Renderer.h"

class PhysWorld
{
public:
    PhysWorld(const Vector3& inGravity)
        : gravity(inGravity)
    {
    }

	void simulate(float deltaTime)
    {
        //integrate unbounded velocities
        for(RigidBody& rb : bodies)
        {
            if(rb.invMass > 0.f)
            {
                rb.linearVelocity += gravity * deltaTime;
            }
        }

        //update constraints

        //solve constraints

        //integrate position
        for (RigidBody& rb : bodies)
        {
            if (rb.invMass > 0.f)
            {
                rb.bodyToWorld.translation += rb.linearVelocity * deltaTime;
                //todo: angular velocity
            }
        }
    }

    int createRigidBody(const Transform& bodyToWorld, const RigidBodyDesc& rigidBodyDesc)
    {
        const int ret = (int)bodies.size();
        bodies.emplace_back(bodyToWorld, rigidBodyDesc);
        return ret;
    }

private:
	std::vector<RigidBody> bodies;
    Vector3 gravity;
    friend class PhysWorldDebugger;
};

class PhysWorldDebugger
{
public:
    PhysWorldDebugger(const PhysWorld& inWorld, Renderer& inRenderer)
        : world(inWorld)
        , renderer(inRenderer)
    {
    }

    void debugDraw() const
    {
        for(const RigidBody& rigidBody : world.bodies)
        {
            renderer.drawOrientedCircles(rigidBody.bodyToWorld, 3.f, 16, 2.f);
            renderer.drawCross(rigidBody.bodyToWorld, 3.f, 2.f);
        }
    }

private:
    const PhysWorld& world;
    Renderer& renderer;
};

#endif