#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "RigidActor.h"
#include "Constraint.h"
#include "GJK.h"

TEST_CASE("GeometryTests", "[physics]")
{
   Vector3 a(10.f, 11.f, 12.f);
   Vector3 b(20.f, 11.f, 12.f);

   REQUIRE(Vector3::isNearlyEqual(a, getClosestPointOnLineSegment(a,b,a)));
   REQUIRE(Vector3::isNearlyEqual(b, getClosestPointOnLineSegment(a,b,b)));
   REQUIRE(Vector3::isNearlyEqual(Vector3(15.f, 11.f, 12.f), getClosestPointOnLineSegment(a,b,Vector3(15.f, 11.f, 12.f))));

   Vector3 c(20.f, 20.f, 12.f);
   REQUIRE(Vector3::isNearlyEqual(a, getClosestPointOnTriangle(a,b,c,a)));
   REQUIRE(Vector3::isNearlyEqual(b, getClosestPointOnTriangle(a,b,c,b)));
   REQUIRE(Vector3::isNearlyEqual(c, getClosestPointOnTriangle(a,b,c,c)));
   REQUIRE(Vector3::isNearlyEqual(Vector3(11.f, 11.f, 12.f), getClosestPointOnTriangle(a,b,c,Vector3(11.f, 11.f, 12.f))));
   REQUIRE(Vector3::isNearlyEqual(a, getClosestPointOnTriangle(a,b,b,a)));
   REQUIRE(Vector3::isNearlyEqual(b, getClosestPointOnTriangle(a,b,b,b)));
   REQUIRE(Vector3::isNearlyEqual(b, getClosestPointOnTriangle(b,b,b,b)));
   REQUIRE(Vector3::isNearlyEqual(b, getClosestPointOnTriangle(b,b,b,c)));
   REQUIRE(Vector3::isNearlyEqual(Vector3(15.f, 11.f, 12.f), getClosestPointOnTriangle(a,b,b,Vector3(15.f, 11.f, 12.f))));

   Vector3 d(0.f);
   REQUIRE(Vector3::isNearlyEqual(a, getClosestPointOnTetrahedron(a,b,c,d,a)));
   REQUIRE(Vector3::isNearlyEqual(b, getClosestPointOnTetrahedron(a,b,c,d,b)));
   REQUIRE(Vector3::isNearlyEqual(c, getClosestPointOnTetrahedron(a,b,c,d,c)));
   REQUIRE(Vector3::isNearlyEqual(d, getClosestPointOnTetrahedron(a,b,c,d,d)));
   REQUIRE(Vector3::isNearlyEqual(c, getClosestPointOnTetrahedron(a,b,c,d,Vector3(20.f, 20.f, 20.f))));
   REQUIRE(Vector3::isNearlyEqual(Vector3(15.f, 15.f, 12.f), getClosestPointOnTetrahedron(a,b,c,d,Vector3(15.f, 15.f, 20.f))));
   REQUIRE(Vector3::isNearlyEqual(Vector3(15.f, 15.f, 10.f), getClosestPointOnTetrahedron(a,b,c,d,Vector3(15.f, 15.f, 10.f))));

   Sphere sphere(10.f, Transform::identity());
   REQUIRE(Vector3::isNearlyEqual(support(sphere,Transform(Vector3(1.f, 2.f, 3.f)), Vector3(1.f, 0.f, 0.f)), Vector3(11.f, 2.f, 3.f)));
   REQUIRE(Vector3::isNearlyEqual(support(sphere, Transform(Vector3(1.f, 2.f, 3.f)), Vector3(-1.f, -1.f, 0.f)), Vector3(1.f - 10.f*sqrt(2.f)/2.f, 2.f - 10.f*sqrt(2.f)/2.f, 3.f)));

   Box box(Vector3(1.f, 2.f, 3.f), Transform::identity());
   REQUIRE(Vector3::isNearlyEqual(support(box,Transform(Vector3(10.f, 20.f, 30.f), Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO)), Vector3(1.f, 0.f, 0.f)), Vector3(12.f, 21.f, 27.f)));
   {
       Sphere s1(10.f, Transform::identity());
       Sphere s2(5.f,  Transform::identity());
       REQUIRE(gjkOverlapping(s1,Transform(Vector3(1.f, 0.f, 0.f)), s2, Transform(Vector3(-5.f, 0.f, 0.f))));
       REQUIRE(!gjkOverlapping(s1, Transform(Vector3(10.1f, 0.f, 0.f)), s2, Transform(Vector3(-5.f, 0.f, 0.f))));
       REQUIRE(gjkOverlapping(s1, Transform(Vector3(9.9f, 0.f, 0.f)), s2, Transform(Vector3(-5.f, 0.f, 0.f))));

       Box b1(Vector3(10.f, 20.f, 1.f), Transform::identity());
       REQUIRE(gjkOverlapping(s1,Transform(Vector3(9.9f, 0.f, 0.f)), b1, Transform(Vector3(-10.f, 0.f, 0.f))));
       REQUIRE(!gjkOverlapping(s1,Transform(Vector3(10.1f, 0.f, 0.f)), b1, Transform(Vector3(-10.f, 0.f, 0.f))));

       REQUIRE(gjkOverlapping(s1, Transform(Vector3(51.f, -8.f, 0.f)), b1, Transform(Vector3(21.f, 3.f, 0.f), Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO*0.5f) )));
       REQUIRE(gjkOverlapping(b1, Transform(Vector3(21.f, 3.f, 0.f), Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO*0.5f)), s1, Transform(Vector3(51.f, -8.f, 0.f)) ));
   }
}