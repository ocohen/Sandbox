#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "OCMath.h"
#include "Vector3.h"
#include "Vector4.h"
#include "Quaternion.h"

TEST_CASE("General Math", "[math]")
{
	REQUIRE(isNearlyEqual(1.f, 1.f) == true);
	REQUIRE(isNearlyEqual(1.f, 1.f + 1e-5f) == true);
	REQUIRE(isNearlyEqual(1.f, 1.f + 1e-3f) == false);
	REQUIRE(isNearlyEqual(1.f, 1.f - 1e-5f) == true);
	REQUIRE(isNearlyEqual(1.f, 1.f - 1e-3f) == false);
}

TEST_CASE("Float3", "[math]")
{
	const NFloat<3> a(1.f, 2.f, 3.f);
	const NFloat<3> b(10.f, 20.f, 30.f);

	//REQUIRE(NFloat<3>::isNearlyEqual(b.asNFloat() / c.asNFloat(), NFloat<3>(.1f, .1f, .1f)));
	//REQUIRE(NFloat<3>::isNearlyEqual(b.asNFloat()*c.asNFloat(), NFloat<3>(10.f, 40.f, 90.f)));
}

TEST_CASE("Vector3", "[math]")
{
	Vector3 a(1.f);
	REQUIRE((a.x == 1.f && a.y == 1.f && a.z == 1.f));

	Vector3 b(1.f, 2.f, 3.f);
	REQUIRE((b.x == 1.f && b.y == 2.f && b.z == 3.f));
	REQUIRE(Vector3::isNearlyEqual(b, Vector3(1.f, 2.f, 3.f)));
	REQUIRE(Vector3::isNearlyEqual(b*10.f, Vector3(10.f, 20.f, 30.f)));
	REQUIRE(Vector3::isNearlyEqual(b/10.f, Vector3(.1f, .2f, .3f)));

	Vector3 c(10.f, 20.f, 30.f);
	Vector3 d = a+b;

	REQUIRE(Vector3::isNearlyEqual(b+c, Vector3(11.f, 22.f, 33.f)));
	REQUIRE(Vector3::isNearlyEqual(b-c, Vector3(-9.f, -18.f, -27.f)));

	REQUIRE(isNearlyEqual(Vector3::dotProduct(b,c), 10+40+90));
	REQUIRE(isNearlyEqual(b.length(), sqrt(1.f+4.f+9.f)));
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
	Quaternion rotateZ = Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO);
	Quaternion rotateX = Quaternion::fromAxisAndAngle(Vector3(1.f, 0.f, 0.f), PI);
	Vector3 diag(1.f, 1.f, 0.f);
	Vector3 diagRotatedByZ = rotateZ * diag;
	Vector3 diagRotatedByZThenX = rotateX * diagRotatedByZ;
	REQUIRE(Vector3::isNearlyEqual(diagRotatedByZThenX, Vector3(-1.f, -1.f, 0.f)));
	Vector3 diagRotatedByZThenXSequential = rotateX * rotateZ * diag;
	REQUIRE(Vector3::isNearlyEqual(diagRotatedByZThenX, diagRotatedByZThenX));
}
