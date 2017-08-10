#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "MassProperties.h"

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
