#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "MassProperties.h"
#include "RigidActor.h"
#include "Constraint.h"

TEST_CASE("MassProperties", "[physics]")
{
    const Transform at10x(Vector3(10.f,0.f,0.f), Quaternion(0.f,0.f,0.f,1.f));
    const Transform at10y(Vector3(0.f,10.f,0.f), Quaternion(0.f,0.f,0.f,1.f));
    ShapeUnion sphere(Sphere(3.f, at10x));
    REQUIRE(isNearlyEqual(computeVolume(sphere), 113.097335529f));

    ShapeUnion box(Box(Vector3(2.f, 3.f, 4.f), at10y));
    REQUIRE(isNearlyEqual(computeVolume(box), 192.f));

    MassProperties massProps;
    massProps.addShape(sphere, 10.f);
    REQUIRE(isNearlyEqual(massProps.mass, 1130.97335529f));
    REQUIRE(Vector3::isNearlyEqual(massProps.com, Vector3(10.f,0.f,0.f)));

    massProps.addShape(box, 2.f);
    REQUIRE(isNearlyEqual(massProps.mass, 1514.97335529f));
    REQUIRE(Vector3::isNearlyEqual(massProps.com, Vector3(7.4653019563f,2.534698f,0.f)));

    massProps.addShape(box, 2.f);
    REQUIRE(isNearlyEqual(massProps.mass, 1898.9733529f));
    REQUIRE(Vector3::isNearlyEqual(massProps.com, Vector3(5.95570944774f,4.0442905174f,0.f)));
}

TEST_CASE("RigidActor", "[physics]")
{
    const Vector3 actorLocation(10.f, 11.f, 12.f);
    const Quaternion actorRotation = Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO);
    const Transform actorTM(actorLocation, actorRotation);

    RigidBodyDesc desc;
    desc.shapes.push_back(Sphere(5.f, Transform(Vector3(0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    desc.shapes.push_back(Sphere(5.f, Transform(Vector3(10.f, 0.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    RigidActor actor(actorTM, desc);;

    REQUIRE(Vector3::isNearlyEqual(actor.getWorldTransform().translation, actorLocation));
    REQUIRE(Quaternion::isNearlyEqual(actor.getWorldTransform().rotation, actorRotation));
}

TEST_CASE("Constraint", "[physics]")
{
    const Vector3 b1(0.f);
    RigidBodyDesc simDesc;
    simDesc.shapes.push_back(Sphere(1.f, Transform(b1, Quaternion::identity())));
    
    const Vector3 b2(3.f, 2.f, 0.f);
    RigidBody sim(Transform(b2, Quaternion::identity()), simDesc);
    sim.invInertia = Vector3(1.f);
    sim.invMass = 1.f;

    RigidBodyDesc kinDesc = simDesc;
    kinDesc.invInertia = Vector3(0.f);
    kinDesc.invMass = 0.f;

    RigidBody kin(Transform::identity(), kinDesc);

    const Vector3 r2(-1.f, 0.f, 0.f);
    Constraint c(&kin, Transform::identity(), &sim, Transform(r2, Quaternion::identity()));
    
    sim.angularVelocity = Vector3(0.f, 0.f, 10.f);
    c.prepareConstraint();
    c.solveConstraint(0.f);

    const Vector3 p1 = b1;
    const Vector3 p2 = b2 + r2;
    const Vector3 cN = p2 - p1;
    const Vector3 cNBar = cN.getNormal();
    const float error = Vector3::dotProduct(sim.linearVelocity + Vector3::crossProduct(sim.angularVelocity, r2), cNBar);
    REQUIRE(isNearlyEqual(error, 0.f));
}
