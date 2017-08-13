#ifndef OC_PHYSWORLD_H
#define OC_PHYSWORLD_H

#include <vector>
#include "RigidActor.h"
#include "Constraint.h"
#include "Renderer.h"
#include "ShapeRenderer.h"
#include "Float3.h"

class PhysWorld
{
public:
    PhysWorld(const Vector3& inGravity)
        : gravity(inGravity)
    {
    }

    ~PhysWorld()
    {
        for(Constraint* constraint : constraints)
        {
            delete constraint;
        }

        for(RigidActor* actor : actors)
        {
            delete actor;
        }
    }

	void simulate(float deltaTime)
    {
        //integrate unbounded velocities
        for(RigidActor* actor : actors)
        {
            RigidBody& rb = actor->body;
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
            for (Constraint* constraint : constraints)
            {
                constraint->solveConstraint(invDeltaTime);
            }
        }
        

        //integrate position and apply damping
        for (RigidActor* actor: actors)
        {
            RigidBody& rb = actor->body;
            if (rb.invMass > 0.f)
            {
                //const Float3 linVelScalers = rb.linearVelocity.asScalers<Float3>();
                //rb.linearVelocity = rb.linearVelocity - Vector3(linVelScalers * linVelScalers) * rb.linearDamping;
                rb.linearVelocity = rb.linearVelocity - 0.0001f * Vector3(sign(rb.linearVelocity.x) * rb.linearVelocity.x*rb.linearVelocity.x, sign(rb.linearVelocity.y) * rb.linearVelocity.y*rb.linearVelocity.y, sign(rb.linearVelocity.z) * rb.linearVelocity.z*rb.linearVelocity.z) * rb.linearDamping;
                rb.bodyToWorld.translation += rb.linearVelocity * deltaTime;
                //rb.bodyToWorld.rotation = 0.5f * deltaTime * Quaternion(rb.angularVelocity) * rb.bodyToWorld.rotation;
                float angle = 0.f;
                Vector3 rotationAxis = rb.angularVelocity.getSafeNormal();

                if(isNearlyEqual(rotationAxis.length(), 1.f))
                {
                    angle = rb.angularVelocity.length();
                }

                rb.bodyToWorld.rotation = Quaternion::fromAxisAndAngle(rotationAxis, angle * deltaTime) * rb.bodyToWorld.rotation;    //this is lame, should use quat differentation
                //todo: angular velocity
            }
        }
    }

    int createRigidActor(const Transform& bodyToWorld, const RigidBodyDesc& rigidBodyDesc)
    {
        const int ret = (int)actors.size();
        actors.push_back(new RigidActor(bodyToWorld, rigidBodyDesc));
        //actors[ret]->body.angularVelocity = Vector3(PI, 0.f, 0.f);
        return ret;
    }

    int createConstraint(int actor1, const Transform& localTM1, int actor2, const Transform& localTM2)
    {
        constraints.push_back(new Constraint(&actors[actor1]->body, localTM1, &actors[actor2]->body, localTM2));
        return (int)constraints.size() - 1;
    }

    RigidActor* getActor(int idx) { return actors[idx]; }
    Constraint* getConstraint(int idx){ return constraints[idx]; }

private:
	std::vector<RigidActor*> actors;
    std::vector<Constraint*> constraints;
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
        for(const RigidActor* actor : world.actors)
        {
            const std::vector<ShapeUnion>& shapes = actor->getShapes();
            const Transform actorTM = actor->getWorldTransform();
            const Transform bodyTM = actor->getBodyToWorld();

            renderer.drawOrientedCircles(bodyTM, 3.f, 16, 2.f);
            renderer.drawCross(actorTM, 3.f, 2.f);

            for(const ShapeUnion& shapeUnion : shapes)
            {
                renderShape(shapeUnion.asShape(), bodyTM, renderer);
            }
        }
    }

private:
    const PhysWorld& world;
    Renderer& renderer;
};

#endif
