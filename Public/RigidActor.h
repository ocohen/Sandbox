#ifndef OC_RIGIDACTOR_H
#define OC_RIGIDACTOR_H

class RigidActor
{
public:
    RigidActor(const Transform& worldTM, const RigidBodyDesc& rigidBodyDesc)
    :rigidBody(worldTM, rigidBodyDesc)
    {
        bodyLocalTM = rigidBody.bodyToWorld.relativeTo(worldTM);
    }

    Transform getWorldTransform() const
    {
        return rigidBody.bodyToWorld * bodyLocalTM;
    }

private:
    RigidBody rigidBody;
    Transform bodyLocalTM;  //worldTM = bodyToWorld * bodyLocalTM
};

#endif

