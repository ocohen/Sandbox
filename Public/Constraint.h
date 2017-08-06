#ifndef OC_CONSTRAINT_H
#define OC_CONSTRAINT_H

#include <float.h>
#include "Vector3.h"
#include "Transform.h"

struct Constraint
{
    Constraint()
        : body1(nullptr)
        , body2(nullptr)
        , invMassScale1(1.f)
        , invMassScale2(1.f)
        , invInertiaScale1(1.f)
        , invInertiaScale2(1.f)
        , linearProjection(FLT_MAX)
		, distance(0.f)
    {
    }

    //for now just a simple distance constraint. This is super hacky
    RigidBody* body1;
    RigidBody* body2;
    float invMassScale1;
    float invMassScale2;
    float invInertiaScale1;
    float invInertiaScale2;
    float linearProjection;
    float distance;

    Constraint(RigidBody* inBody1, RigidBody* inBody2)
        : body1(inBody1)
        , body2(inBody2)
        , invMassScale1(1.f)
        , invMassScale2(1.f)
        , invInertiaScale1(1.f)
        , invInertiaScale2(1.f)
        , linearProjection(FLT_MAX)
    {
        distance = (body1->bodyToWorld.translation - body2->bodyToWorld.translation).length();
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

            const Vector3 bToA = body1->bodyToWorld.translation - body2->bodyToWorld.translation;
            const Vector3 bToANormal = bToA.getNormal();
            const float bToALength = bToA.length();
            const float linearError = bToALength - distance;
            if(fabs(linearError) > linearProjection)
            {
                //todo: actually sort by kinematics properly
                body1->bodyToWorld.translation += -bToANormal * weight1 * linearError;
                body2->bodyToWorld.translation += bToANormal * weight2 * linearError;
            }
            else
            {
                const Vector3 relVelocity = bToANormal * (Vector3::dotProduct(body1->linearVelocity - body2->linearVelocity, bToANormal) + linearError*invDeltaTime * 0.3f);
                body1->linearVelocity += -relVelocity * weight1;
                body2->linearVelocity += relVelocity * weight2;
            }
        }
    }
    
};

#endif
