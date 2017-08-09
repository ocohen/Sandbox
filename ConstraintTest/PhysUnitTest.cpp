#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "RigidBody.h"

TEST_CASE("Volume", "[physics]")
{
    const Transform identity(Vector3(0.f,0.f,0.f), Quaternion(0.f,0.f,0.f,1.f));
    ShapeUnion sphere(Sphere(3.f, identity));
    REQUIRE(isNearlyEqual(computeVolume(sphere), 113.097335529f));

    ShapeUnion box(Box(Vector3(2.f, 3.f, 4.f), identity));
    REQUIRE(isNearlyEqual(computeVolume(box), 192.f));
}
