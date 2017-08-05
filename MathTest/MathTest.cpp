#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "OCMath.h"
#include "Vector3.h"
#include "Vector4.h"
#include "Quaternion.h"
#include "Float3.h"
#include "Transform.h"

static_assert(sizeof(Vector3) == 12, "Vector3 assumes to be tightly packed");
static_assert(sizeof(Vector4) == 16, "Vector4 assumes to be tightly packed");
static_assert(sizeof(Quaternion) == 16, "Quaternion assumes to be tightly packed");

TEST_CASE("General Math", "[math]")
{
    REQUIRE(isNearlyEqual(1.f, 1.f) == true);
    REQUIRE(isNearlyEqual(1.f, 1.f + 1e-5f) == true);
    REQUIRE(isNearlyEqual(1.f, 1.f + 1e-3f) == false);
    REQUIRE(isNearlyEqual(1.f, 1.f - 1e-5f) == true);
    REQUIRE(isNearlyEqual(1.f, 1.f - 1e-3f) == false);
}

TEST_CASE("Scaler3", "[math]")
{
    Float3 a(1.f, 2.f, 3.f);
    Float3 b(2.f, 3.f, 4.f);
    a *= b;
    REQUIRE(Float3::isNearlyEqual(a, Float3(2.f, 6.f, 12.f)));

    Int3 x(1,2,3);
    x += 1;
    REQUIRE(Int3::isNearlyEqual(x, Int3(2,3,4)));
}

TEST_CASE("Vector3", "[math]")
{
    Vector3 a(1.f);
    REQUIRE((a.x == 1.f && a.y == 1.f && a.z == 1.f));

    Vector3 b(1.f, 2.f, 3.f);
    REQUIRE((b.x == 1.f && b.y == 2.f && b.z == 3.f));
    REQUIRE(Vector3::isNearlyEqual(b, Vector3(1.f, 2.f, 3.f)));
    REQUIRE(Vector3::isNearlyEqual(b*10.f, Vector3(10.f, 20.f, 30.f)));

    Vector3 c(10.f, 20.f, 30.f);
    Vector3 d = a+b;

    REQUIRE(Vector3::isNearlyEqual(b+c, Vector3(11.f, 22.f, 33.f)));
    REQUIRE(Vector3::isNearlyEqual(b-c, Vector3(-9.f, -18.f, -27.f)));

    REQUIRE(isNearlyEqual(Vector3::dotProduct(b,c), 10.f+40.f+90.f));
    REQUIRE(isNearlyEqual(b.length(), sqrtf(1.f+4.f+9.f)));
    REQUIRE(isNearlyEqual(b.length2(), 1.f+4.f+9.f));
    REQUIRE(Vector3::isNearlyEqual(b.getNormal(), Vector3(1.f / 3.74165738677f, 2.f / 3.74165738677f, 3.f / 3.74165738677f)));

    Vector3 x(1,0,0);
    Vector3 y(0,1,0);
    Vector3 z = Vector3::crossProduct(x,y);
    REQUIRE(Vector3::isNearlyEqual(z, Vector3(0,0,1)));

    Vector3 test;
    test = x * 3;
}

TEST_CASE("Vector4", "[math]")
{
    Vector4 u(1, 2, 3, 0);
    Vector4 v(0,0,0,1);
    REQUIRE(isNearlyEqual(Vector4::dotProduct(u,v), 0.f));
}

TEST_CASE("Quaternion", "[math]")
{
    Quaternion r = Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO);

    float angle;
    Vector3 axis;
    r.toAxisAngle(axis, angle);
    REQUIRE(angle == PI_OVER_TWO);
    REQUIRE(Vector3::isNearlyEqual(axis, Vector3(0.f, 0.f, 1.f)));

    Vector3 v(0.f, 0.f, 10.f);
    Vector3 vRotated = r * v;
    REQUIRE(Vector3::isNearlyEqual(v, vRotated));
    Vector3 uRotated = r * Vector3(1.f, 1.f, 0.f);
    REQUIRE(Vector3::isNearlyEqual(Vector3(-1.f, 1.f, 0.f), uRotated));
    REQUIRE(Vector3::isNearlyEqual(Vector3(1.f, 1.f, 0.f), r.getInverse() * uRotated));
    REQUIRE(Vector3::isNearlyEqual(r * r.getInverse() * v,v));
    REQUIRE(Vector3::isNearlyEqual(r * r * Vector3(1.f, 1.f, 0.f), Vector3(-1.f, -1.f, 0.f)));

    //make sure sequence of rotation maintains proper order
    Quaternion rotatez = Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO);
    Quaternion rotatex = Quaternion::fromAxisAndAngle(Vector3(1.f, 0.f, 0.f), PI);
    Vector3 diag(1.f, 1.f, 0.f);
    Vector3 diagRotatedByz = rotatez * diag;
    Vector3 diagRotatedByzThenx = rotatex * diagRotatedByz;
    REQUIRE(Vector3::isNearlyEqual(diagRotatedByzThenx, Vector3(-1.f, -1.f, 0.f)));
    Vector3 diagRotatedByzThenxSequential = rotatex * rotatez * diag;
    REQUIRE(Vector3::isNearlyEqual(diagRotatedByzThenx, diagRotatedByzThenx));
    REQUIRE(isNearlyEqual(rotatez.length(), 1.f));
    REQUIRE(isNearlyEqual((rotatex*rotatez).length(), 1.f));

    //make sure doubles work
    QuaternionD rD = QuaternionD::fromAxisAndAngle(Vector3d(0.f, 0.f, 1.f), PI_OVER_TWO);
    double angleD;
    Vector3d axisD;
    rD.toAxisAngle(axisD, angleD);
    REQUIRE(angleD == PI_OVER_TWO);
    REQUIRE(Vector3d::isNearlyEqual(axisD, Vector3d(0.f, 0.f, 1.f)));

}

TEST_CASE("Transform", "[math]")
{
    const Transform translate(Vector3(10.f, 11.f, 12.f), Quaternion::fromAxisAndAngle(Vector3(1.f, 0.f, 0.f), 0.f));
    const Transform rotate(Vector3(0.f, 0.f, 0.f), Quaternion::fromAxisAndAngle(Vector3(1.f, 0.f, 0.f), PI_OVER_TWO));
    const Transform tm(Vector3(10.f, 11.f, 12.f), Quaternion::fromAxisAndAngle(Vector3(1.f, 0.f, 0.f), PI_OVER_TWO));

    //origin
    {
        const Vector3 origin(0.f, 0.f, 0.f);
        REQUIRE(Vector3::isNearlyEqual(translate.transformPoint(origin), Vector3(10.f, 11.f, 12.f)));
        REQUIRE(Vector3::isNearlyEqual(translate.transformVector(origin), Vector3(0.f, 0.f, 0.f)));

        REQUIRE(Vector3::isNearlyEqual(rotate.transformPoint(origin), Vector3(0.f, 0.f, 0.f)));
        REQUIRE(Vector3::isNearlyEqual(rotate.transformVector(origin), Vector3(0.f, 0.f, 0.f)));

        REQUIRE(Vector3::isNearlyEqual(tm.transformPoint(origin), Vector3(10.f, 11.f, 12.f)));
        REQUIRE(Vector3::isNearlyEqual(tm.transformVector(origin), Vector3(0.f, 0.f, 0.f)));
    }

    //off-origin
    {
        const Vector3 x(1.f, 2.f, 3.f);
        REQUIRE(Vector3::isNearlyEqual(translate.transformPoint(x), Vector3(11.f, 13.f, 15.f)));
        REQUIRE(Vector3::isNearlyEqual(translate.transformVector(x), Vector3(1.f, 2.f, 3.f)));

        REQUIRE(Vector3::isNearlyEqual(rotate.transformPoint(x), Vector3(1.f, -3.f, 2.f)));
        REQUIRE(Vector3::isNearlyEqual(rotate.transformVector(x), Vector3(1.f, -3.f, 2.f)));

        REQUIRE(Vector3::isNearlyEqual(tm.transformPoint(x), Vector3(11.f, 8.f, 14.f)));
        REQUIRE(Vector3::isNearlyEqual(tm.transformVector(x), Vector3(1.f, -3.f, 2.f)));
    }

    //concat
    {
        const Vector3 x(1.f, 2.f, 3.f);
        const Transform rotateZAndMoveX(Vector3(10.f, 0.f, 0.f), Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO));
        const Transform rotateZAndMoveY(Vector3(0.f, 10.f, 0.f), Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), -PI));
        const Transform compound = rotateZAndMoveY * rotateZAndMoveX;

        REQUIRE(Vector3::isNearlyEqual(compound.transformPoint(x), rotateZAndMoveY.transformPoint(rotateZAndMoveX.transformPoint(x))));
        REQUIRE(Vector3::isNearlyEqual(compound.transformPoint(x), Vector3(-8.f, 9.f, 3.f)));
    }
}
