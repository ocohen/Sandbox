#ifndef OC_RIGIDACTOR_H
#define OC_RIGIDACTOR_H

#include "RigidBody.h"

class RigidActor
{
public:
    RigidActor(const Transform& worldTM, const RigidBodyDesc& bodyDesc)
    :body(worldTM, bodyDesc)
    {
        bodyLocalTM = body.bodyToWorld.inverseTransform(worldTM);
    }

    Transform getWorldTransform() const
    {
        return body.bodyToWorld * bodyLocalTM;
    }

    void setWorldTransform(const Transform& worldTM)
    {
        //current = body.bodyToWorld * bodyLocalTM
        //new = body.bodyToWorld' * bodyLocalTM
        //=> body.bodyToWorld' = new * bodyLocalTM.inv
        body.bodyToWorld = worldTM.inverseTransformReverse(bodyLocalTM);
    }

    const std::vector<ShapeUnion>& getShapes() const
    {
        return body.shapes;
    }

    void setLinearVelocity(const Vector3& inLinVel)
    {
        body.linearVelocity = inLinVel;
    }

    const Vector3& getLinearVelocity() const
    {
        return body.linearVelocity;
    }

    void setAngularVelocity(const Vector3& inAngVel)
    {
        body.angularVelocity = inAngVel;
    }

    const Vector3& getAngularVelocity() const
    {
        return body.angularVelocity;
    }

private:
    friend class PhysWorld;
    friend class PhysWorldDebugger;
    RigidBody body;
    Transform bodyLocalTM;  //worldTM = bodyToWorld * bodyLocalTM

    const Transform& getBodyToWorld() const 
    {
        return body.bodyToWorld;
    }
};

#endif

