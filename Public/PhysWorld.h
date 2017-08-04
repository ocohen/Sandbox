#ifndef OC_PHYSWORLD_H
#define OC_PHYSWORLD_H

#include <vector>
#include "RigidBody.h"
#include "Constraint.h"
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

        const float invDeltaTime = 1.f / deltaTime;

        //solve constraints
        for(int itr = 0; itr < 4; ++itr)
        {
            for (Constraint& constraint : constraints)
            {
                constraint.solveConstraint(invDeltaTime);
            }
        }
        

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

    int createConstraint(int body1, int body2)
    {
        constraints.emplace_back(&bodies[body1], &bodies[body2]);
        return (int)constraints.size() - 1;
    }

    RigidBody& getBody(int idx) { return bodies[idx]; }
    Constraint& getConstraint(int idx){ return constraints[idx]; }

private:
	std::vector<RigidBody> bodies;
    std::vector<Constraint> constraints;
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