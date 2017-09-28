#ifndef OC_PHYSWORLD_H
#define OC_PHYSWORLD_H

#include <vector>
#include "RigidActor.h"
#include "Constraint.h"
#include "Renderer.h"
#include "ShapeRenderer.h"
#include "Float3.h"
#include "Vector4.h"
#include "Logger.h"
#include <sstream>
#include "RigidBody.h"
#include "GJK.h"

struct ContactCache
{
    RigidBody* bodyA;
    RigidBody* bodyB;

    Transform contactsA[4];
    Transform contactsB[4];
    Vector3 normals[4]; //TODO: this needs to be local to A
    int contactCountdown[4];

    ContactCache(RigidBody* inA, RigidBody* inB)
        : bodyA(inA)
        , bodyB(inB)
        , timeToDropContact(2)
    {
        for(int& x : contactCountdown)
        {
            x = 0;
        }
    }

    bool areWithinThreshold(const Transform& x, const Transform& y)
    {
        return false;//(x.translation - y.translation).length2() < (0.1f * 0.1f);   //TODO: this isn't quite right for rotations
    }

    static constexpr int numContactsPossible() 
    {
        return sizeof(contactsA) / sizeof(contactsA[0]);
    }

    int getNextIndex() const
    {
        int retIdx = 0;
        int minCount = INT_MAX;
        for(int i=0; i< numContactsPossible(); ++i)
        {
            if(contactCountdown[i] < minCount)
            {
                minCount = contactCountdown[i];
                retIdx = i;
            }
        }
        
        return retIdx;
    }

    void addContactPair(const Transform& localAPt, const Transform& localBPt, const Vector3& normal)
    {
        //TODO: doesn't handle local normal
        //first check if it's a new point
        for(int i=0; i<numContactsPossible(); ++i)
        {
            if(contactCountdown[i] > 0)
            {
                const bool aMatches = areWithinThreshold(localAPt, contactsA[i]);
                const bool bMatches = areWithinThreshold(localBPt, contactsB[i]);

                if (aMatches && bMatches)
                {
                    contactCountdown[i] = timeToDropContact;
                    return;
                }
                else if (aMatches)
                {
                    contactsA[i] = localAPt;
                    contactsB[i] = localBPt;
                    contactCountdown[i] = timeToDropContact;
                    return;
                }
                else if (bMatches)
                {
                    contactsA[i] = localAPt;
                    contactsB[i] = localBPt;
                    contactCountdown[i] = timeToDropContact;
                    return;
                }
            }
        }

        int nextIdx = getNextIndex();

        //actually a new contact point
        contactsA[nextIdx] = localAPt;
        contactsB[nextIdx] = localBPt;
        normals[nextIdx] = normal;
        contactCountdown[nextIdx] = timeToDropContact;
    }

    void tickCache()
    {
        for(int i=0; i< numContactsPossible(); ++i)
        {
            if(contactCountdown[i] > 0)
            {
                --contactCountdown[i];
            }
        }
    }

private:
    int timeToDropContact;
};

class PhysWorld
{
public:
    PhysWorld(const Vector3& inGravity)
        : gravity(inGravity)
        , logger(nullptr)
    {
        fixedConstraints = false;
    }

    bool fixedConstraints;

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

        //generate contacts
        if(true)
        {
            for(int i=0; i<actors.size(); ++i)
            {
                for(int j=i+1; j<actors.size(); ++j)
                {
                    //todo: compound shapes
                    RigidBody* bodyA = &actors[i]->body;
                    RigidBody* bodyB = &actors[j]->body;

                    if(bodyA->invMass <= OC_BIG_EPSILON && bodyB->invMass <= OC_BIG_EPSILON)
                    {
                        continue;
                    }

                    if(gjkOverlapping(bodyA->shapes[0], bodyA->bodyToWorld * bodyA->shapes[0].asShape().localTM, bodyB->shapes[0], bodyB->bodyToWorld * bodyB->shapes[0].asShape().localTM, 2.f))
                    {
                        GJKInfo info;

                        if (gjkGetClosestPoints<true>(bodyA->shapes[0], bodyA->bodyToWorld * bodyA->shapes[0].asShape().localTM, bodyB->shapes[0], bodyB->bodyToWorld * bodyB->shapes[0].asShape().localTM, nullptr, 0.f, info))
                        {
                            const Transform localA = bodyA->bodyToWorld.inverseTransform(info.closestA);
                            const Transform localB = bodyB->bodyToWorld.inverseTransform(info.closestB);

                            ContactCache* useCache = nullptr;
                            for(ContactCache& cache : contactCaches)
                            {
                                if(cache.bodyA == bodyA && cache.bodyB == bodyB)
                                {
                                    useCache = &cache;
                                    break;
                                }
                            }

                            if(!useCache)
                            {
                                contactCaches.emplace_back(bodyA, bodyB);
                                useCache = &contactCaches.back();
                            }

                            useCache->addContactPair(localA, localB, info.aToBNormal);
                        }
                    }
                }
            }
        }

        //TODO: this is super hacky
        for (Constraint* c : contactConstraints)
        {
            delete c;
        }

        //generate constraints per contact
        contactConstraints.clear();

        for(ContactCache& cache : contactCaches)
        {
            RigidBody* bodyA = cache.bodyA;
            RigidBody* bodyB = cache.bodyB;

            for(int pairIdx=0; pairIdx<cache.numContactsPossible(); ++pairIdx)
            {
                if(cache.contactCountdown[pairIdx] == 0)
                {
                    continue;
                }

                const Transform& localA = cache.contactsA[pairIdx];
                const Transform& localB = cache.contactsB[pairIdx];
                const Vector3& normal = cache.normals[pairIdx]; //TODO: this is not localized properly

                Constraint* newConstraint = new Constraint(bodyA, localA, bodyB, localB);
                newConstraint->distance = 2.f;
                newConstraint->prepareConstraint();
                newConstraint->normals[0] = normal;
                newConstraint->minImpulse = 0.f;
                newConstraint->baumgarte = 0.01f;
                contactConstraints.push_back(newConstraint);

                const float grav = gravity.length();
                const float massA = bodyA->invMass > 0.f ? 1.f / bodyA->invMass : 0.f;
                const float massB = bodyB->invMass > 0.f ? 1.f / bodyB->invMass : 0.f;
                float gravMass = grav * (massA + massB) * 0.01f;
                Vector3 u, v;
                computeBasis(normal, u, v);
                Constraint* fric1 = new Constraint(bodyA, localA, bodyB, localB);
                fric1->distance = 2.f;
                fric1->prepareConstraint();
                fric1->normals[0] = u;
                fric1->minImpulse = -gravMass;
                fric1->maxImpulse = gravMass;
                fric1->baumgarte = 0.0f;
                contactConstraints.push_back(fric1);

                Constraint* fric2 = new Constraint(bodyA, localA, bodyB, localB);
                fric2->distance = 2.f;
                fric2->prepareConstraint();
                fric2->normals[0] = v;
                fric2->minImpulse = -gravMass;
                fric2->maxImpulse = gravMass;
                fric2->baumgarte = 0.0f;
                contactConstraints.push_back(fric2);
            }
        }

        //update constraints
        for (Constraint* constraint : constraints)
        {
            constraint->prepareConstraint();
        }

        const float invDeltaTime = 1.f / deltaTime;

        //solve constraints
        for(int itr = 0; itr < 8; ++itr)
        {
            if (logger)
            {
                logger->advance();
            }

            for (Constraint* constraint : constraints)
            {
                constraint->solveConstraint(invDeltaTime);
            }

            for (Constraint* constraint : contactConstraints)
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

                rb.bodyToWorld.rotation = Quaternion::fromAxisAndAngle(rotationAxis, angle * deltaTime) * rb.bodyToWorld.rotation;    //this is lame, should use quat differentation*/

                /*Vector3 imag = rb.bodyToWorld.rotation.imaginary;
                Vector4 quat(imag.x, imag.y, imag.z, rb.bodyToWorld.rotation.real);
                Vector4 leftQuat = quat + quat * deltaTime * 0.5f;
                Quaternion leftQuatq(leftQuat.x, leftQuat.y, leftQuat.z, leftQuat.w);
                Quaternion rightQuat(rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z, 0.f);
                Quaternion finalQ = leftQuatq * rightQuat;
                rb.bodyToWorld.rotation = finalQ;*/
            }
        }

        for(ContactCache& cache : contactCaches)
        {
            cache.tickCache();
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
        RigidBody* rigidBody1 = actor1 == -1 ? nullptr : &actors[actor1]->body;
        RigidBody* rigidBody2 = actor2 == -1 ? nullptr : &actors[actor2]->body;

        Constraint* newConstraint = fixedConstraints ? new PerAxisConstraint(rigidBody1, localTM1, rigidBody2, localTM2) : new Constraint(rigidBody1, localTM1, rigidBody2, localTM2);

        constraints.push_back(newConstraint);
        const int constraintIdx = (int)constraints.size() - 1;

        if(logger && constraints.size() == 1)
        {
            std::ostringstream stringStream;
            stringStream << "Constraint[" << constraintIdx << "]";
            std::string copyOfStr = stringStream.str();

            newConstraint->logger = logger;
            newConstraint->loggerKey = stringStream.str();
        }

        return constraintIdx;
    }

    RigidActor* getActor(int idx) { return actors[idx]; }
    Constraint* getConstraint(int idx){ return constraints[idx]; }

    Logger* logger;

    const std::vector<Constraint*>& getContactConstraints(){ return contactConstraints; }

private:
	std::vector<RigidActor*> actors;
    std::vector<Constraint*> constraints;
    std::vector<Constraint*> contactConstraints;
    std::vector<ContactCache> contactCaches;
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
