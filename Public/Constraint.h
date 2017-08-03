#ifndef OC_CONSTRAINT_H
#define OC_CONSTRAINT_H

#include "Vector3.h"
#include "Transform.h"

struct Constraint
{
    //for now just a simple distance constraint. This is super hacky
    RigidBody* body1;
    RigidBody* body2;

    void solveConstraint()
    {
        if(body1->invMass > OC_BIG_EPSILON || body2->invMass > OC_BIG_EPSILON)
        {
            const Vector3 bToA = body1->bodyToWorld.translation - body2->bodyToWorld.translation;
            const Vector3 relVelocity = bToA.getNormal() * Vector3::dotProduct(body1->linearVelocity - body2->linearVelocity, bToA.getNormal());
            const float totalMass = 1.f / (body1->invMass + body2->invMass);
            const float weight1 = totalMass * body1->invMass; 
            const float weight2 = totalMass * body2->invMass;
            body1->linearVelocity += -relVelocity * weight1;
            body2->linearVelocity += relVelocity * weight2;
        }
        

    }
    
};

#endif