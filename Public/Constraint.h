#ifndef OC_CONSTRAINT_H
#define OC_CONSTRAINT_H

#include <float.h>
#include "Vector3.h"
#include "Transform.h"

struct Constraint
{
    //for now just a simple distance constraint. This is super hacky
    RigidBody* body1;
    RigidBody* body2;
    Transform frame1;
    Transform frame2;
    float invMassScale1;
    float invMassScale2;
    float invInertiaScale1;
    float invInertiaScale2;
    float linearProjection;
    float distance;

    Constraint(RigidBody* inBody1, const Transform& localTM1, RigidBody* inBody2, const Transform& localTM2)
        : body1(inBody1)
        , body2(inBody2)
        , invMassScale1(1.f)
        , invMassScale2(1.f)
        , invInertiaScale1(1.f)
        , invInertiaScale2(1.f)
        , linearProjection(FLT_MAX)
    {
        frame1 = localTM1;
        frame2 = localTM2;

        distance = ((body1->bodyToWorld * frame1).translation - (body2->bodyToWorld * frame2).translation).length();
    }

    void solveConstraint(float invDeltaTime)
    {
        if(body1->invMass > OC_BIG_EPSILON || body2->invMass > OC_BIG_EPSILON)
        {
            const float invMass1 = body1->invMass * invMassScale1;
            const float invMass2 = body2->invMass * invMassScale2;
            const float totalMass = 1.f / (invMass1 + invMass2);
            const float weight1 = totalMass * invMass1;
            const float weight2 = totalMass * invMass2;

            const Transform& body1TM = body1->bodyToWorld;
            const Transform p1TM = body1TM * frame1;
            const Transform& body2TM = body2->bodyToWorld;
            const Transform p2TM = body2TM * frame2;


            const Vector3 p2Top1 = p1TM.translation - p2TM.translation;
            const Vector3 p2ToP1Normal = p2Top1.getSafeNormal();
            const float p2ToP1Length = p2Top1.length();
            const float linearError = p2ToP1Length - distance;
            /*if(fabs(linearError) > linearProjection)
            {
                //todo: actually sort by kinematics properly
                //body1->bodyToWorld.translation += -p2ToP1Normal * weight1 * linearError;
                //body2->bodyToWorld.translation += p2ToP1Normal * weight2 * linearError;
            }
            else*/
            {
                const Vector3 comToP1 = p1TM.translation - body1TM.translation;
                const Vector3 p1WorldVel = body1->linearVelocity + Vector3::crossProduct(body1->angularVelocity, comToP1);

                const Vector3 comToP2 = p2TM.translation - body2TM.translation;
                const Vector3 p2WorldVel = body2->linearVelocity + Vector3::crossProduct(body2->angularVelocity, comToP2);

                const float relVelocity = Vector3::dotProduct(p1WorldVel - p2WorldVel, p2ToP1Normal);
                const Vector3 correctionImpulse = p2ToP1Normal * (relVelocity + linearError*invDeltaTime * 0.3f);
                
                const Vector3 b1LinearImpulse = -correctionImpulse * weight1;
                body1->linearVelocity += b1LinearImpulse;
                body1->angularVelocity += Vector3::crossProduct(comToP1, b1LinearImpulse);

                const Vector3 b2LinearImpulse = correctionImpulse * weight2;
                body2->linearVelocity += b2LinearImpulse;
                body2->angularVelocity += Vector3::crossProduct(comToP2, b2LinearImpulse);
            }
        }
    }
    
};

#endif
