#ifndef OC_CONSTRAINT_H
#define OC_CONSTRAINT_H

#include <float.h>
#include "Vector3.h"
#include "Transform.h"
#include "Logger.h"

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

    //prepared data
    std::vector<Vector3> normals;
    std::vector<float> geometricErrors;
    Vector3 r1;
    Vector3 r2;
    float weight1;
    float weight2;
    float invMass1;
    float invMass2;

    float maxImpulse;
    float minImpulse;
    float accumulatedImpulse;
    float baumgarte;

    Logger* logger;
    std::string loggerKey;

    Constraint(RigidBody* inBody1, const Transform& localTM1, RigidBody* inBody2, const Transform& localTM2)
        : body1(inBody1)
        , body2(inBody2)
        , frame1(localTM1)
        , frame2(localTM2)
        , invMassScale1(1.f)
        , invMassScale2(1.f)
        , invInertiaScale1(1.f)
        , invInertiaScale2(1.f)
        , linearProjection(FLT_MAX)
        , maxImpulse(FLT_MAX)
        , minImpulse(-FLT_MAX)
        , accumulatedImpulse(0.f)
        , baumgarte(1.f)
        , logger(nullptr)
    {
        const Transform body1TM = body1 ? body1->bodyToWorld : Transform::identity();
        const Transform p1TM = body1TM * frame1;
        const Transform body2TM = body2 ? body2->bodyToWorld : Transform::identity();
        const Transform p2TM = body2TM * frame2;

        distance = (p1TM.translation - p2TM.translation).length();
    }

    virtual void prepareConstraint()
    {
        invMass1 = body1 ? body1->invMass * invMassScale1 : 0.f;
        invMass2 = body2 ? body2->invMass * invMassScale2 : 0.f;
        accumulatedImpulse = 0.f;

        if(invMass1 > OC_BIG_EPSILON || invMass2 > OC_BIG_EPSILON)
        {
            const float totalMass = 1.f / (invMass1 + invMass2);

            weight1 = totalMass * invMass1;
            weight2 = totalMass * invMass2;

            const Transform body1TM = body1 ? body1->bodyToWorld : Transform::identity();
            const Transform p1TM = body1TM * frame1;
            const Transform body2TM = body2 ? body2->bodyToWorld : Transform::identity();
            const Transform p2TM = body2TM * frame2;


            const Vector3 n = p2TM.translation - p1TM.translation;
            normals.clear();
            normals.push_back(n.getSafeNormal());
            const float nLength = n.length();
            geometricErrors.clear();
            geometricErrors.push_back(distance - nLength);

             r1 = p1TM.translation - body1TM.translation;
             r2 = p2TM.translation - body2TM.translation;
        }
    }

    void solveConstraint(float invDeltaTime)
    {
        if (invMass1 > OC_BIG_EPSILON || invMass2 > OC_BIG_EPSILON)
        {
            const int numRows = (int)normals.size();
            for(int rowIdx = 0; rowIdx < numRows; ++rowIdx)
            {
                const Vector3& normal = normals[rowIdx];
                const float geometricError = geometricErrors[rowIdx];

                if (logger)
                {
                    logger->log(loggerKey, std::string("linearError"), geometricError);
                }
                const Vector3 p1WorldVel = body1 ? body1->linearVelocity + Vector3::crossProduct(body1->angularVelocity, r1) : Vector3(0.f);
                const Vector3 p2WorldVel = body2 ? body2->linearVelocity + Vector3::crossProduct(body2->angularVelocity, r2) : Vector3(0.f);
                const float relVelocity = Vector3::dotProduct(p2WorldVel - p1WorldVel, normal);

                if (logger)
                {
                    logger->log(loggerKey, std::string("relVelocity"), relVelocity);
                }

                const float r2squaredMinusR2Dot = r2.length2()*weight2 - r2.dotProduct(normal*weight2)*r2.dotProduct(normal);
                const float r1squaredMinusR1Dot = r1.length2()*weight1 - r1.dotProduct(normal*weight1)*r1.dotProduct(normal);
                const float lambda = -relVelocity / (1.f + r2squaredMinusR2Dot + r1squaredMinusR1Dot);
                const float computedImpulse = (lambda + geometricError * invDeltaTime * baumgarte);
                const float prevAccumulatedImpulse = accumulatedImpulse;
                const float newTotalImpulse = clamp(prevAccumulatedImpulse + computedImpulse, minImpulse, maxImpulse);
                const float finalComputedImpulse = newTotalImpulse - prevAccumulatedImpulse;
                accumulatedImpulse = newTotalImpulse;

                if(!isNearlyEqual(finalComputedImpulse, OC_BIG_EPSILON))
                {
                    const Vector3 correctionImpulse = finalComputedImpulse * normal;

                    if (logger)
                    {
                        logger->log(loggerKey, std::string("lambda"), -lambda);
                    }

                    if (invMass1 > 0)
                    {
                        const Vector3 b1LinearImpulse = -correctionImpulse * weight1 * (1.f / invMass1);  //gross, compute better above
                        body1->linearVelocity += b1LinearImpulse * invMass1;
                        body1->angularVelocity += Vector3::crossProduct(r1, b1LinearImpulse) * body1->invInertia.x;    //terrible assumes completely symmetric
                    }

                    if (invMass2 > 0)
                    {
                        const Vector3 b2LinearImpulse = correctionImpulse * weight2 * (1.f / invMass2);
                        body2->linearVelocity += b2LinearImpulse * invMass2;
                        body2->angularVelocity += Vector3::crossProduct(r2, b2LinearImpulse) * body2->invInertia.x;   //terrible assumes completely symmetric
                    }
                }
            }
        }
    }
};

struct PerAxisConstraint : public Constraint
{
    PerAxisConstraint(RigidBody* inBody1, const Transform& localTM1, RigidBody* inBody2, const Transform& localTM2)
        : Constraint(inBody1, localTM1, inBody2, localTM2)
    {
        const Transform body1TM = body1 ? body1->bodyToWorld : Transform::identity();
        const Transform p1TM = body1TM * frame1;
        const Transform body2TM = body2 ? body2->bodyToWorld : Transform::identity();
        const Transform p2TM = body2TM * frame2;

        Vector3 d = p2TM.translation - p1TM.translation;
        distances[0] = d.x;
        distances[1] = d.y;
        distances[2] = d.z;
    }

    virtual void prepareConstraint() override
    {
        invMass1 = body1 ? body1->invMass * invMassScale1 : 0.f;
        invMass2 = body2 ? body2->invMass * invMassScale2 : 0.f;
        accumulatedImpulse = 0.f;

        if (invMass1 > OC_BIG_EPSILON || invMass2 > OC_BIG_EPSILON)
        {
            const float totalMass = 1.f / (invMass1 + invMass2);

            weight1 = totalMass * invMass1;
            weight2 = totalMass * invMass2;

            const Transform body1TM = body1 ? body1->bodyToWorld : Transform::identity();
            const Transform p1TM = body1TM * frame1;
            const Transform body2TM = body2 ? body2->bodyToWorld : Transform::identity();
            const Transform p2TM = body2TM * frame2;

            if(normals.size() == 0)
            {
                normals.push_back(Vector3(1.f, 0.f, 0.f));
                normals.push_back(Vector3(0.f, 1.f, 0.f));
                normals.push_back(Vector3(0.f, 0.f, 1.f));
            }
            
            const Vector3 offset = p2TM.translation - p1TM.translation;
           
            geometricErrors.clear();
            for(int i=0; i<3; ++i)
            {
                geometricErrors.push_back((distances[i] - offset.dotProduct(normals[i])));
            }

            r1 = p1TM.translation - body1TM.translation;
            r2 = p2TM.translation - body2TM.translation;
        }
    }

    float distances[3];
};

#endif
