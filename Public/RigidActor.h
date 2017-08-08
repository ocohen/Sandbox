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

    RigidBody body;
    Transform bodyLocalTM;  //worldTM = bodyToWorld * bodyLocalTM
};

#endif

