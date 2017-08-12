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

    const std::vector<ShapeUnion>& getShapes() const
    {
        return body.shapes;
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

