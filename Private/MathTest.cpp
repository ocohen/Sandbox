#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "OCMath.h"
#include "Vector3.h"
#include "Vector4.h"

TEST_CASE("General Math", "[math]")
{
	REQUIRE(isNearlyEqual(1.f, 1.f) == true);
	REQUIRE(isNearlyEqual(1.f, 1.f + 1e-8f) == true);
	REQUIRE(isNearlyEqual(1.f, 1.f + 1e-7f) == false);
	REQUIRE(isNearlyEqual(1.f, 1.f - 1e-8f) == true);
	REQUIRE(isNearlyEqual(1.f, 1.f - 1e-7f) == false);
}

TEST_CASE("Vector operations", "[math]")
{
	Vector3 a(1.f);
	REQUIRE((a.x == 1.f && a.y == 1.f && a.z == 1.f));

	Vector3 b(1.f, 2.f, 3.f);
	REQUIRE((b.x == 1.f && b.y == 2.f && b.z == 3.f));
	REQUIRE(Vector3::isNearlyEqual(b, Vector3(1.f, 2.f, 3.f)));
	REQUIRE(Vector3::isNearlyEqual(b*10.f, Vector3(10.f, 20.f, 30.f)));
	REQUIRE(Vector3::isNearlyEqual(b/10.f, Vector3(.1f, .2f, .3f)));

	Vector3 c(10.f, 20.f, 30.f);

	REQUIRE(Vector3::isNearlyEqual(b+c, Vector3(11.f, 22.f, 33.f)));
	REQUIRE(Vector3::isNearlyEqual(b-c, Vector3(-9.f, -18.f, -27.f)));
	REQUIRE(Vector3::isNearlyEqual(b*c, Vector3(10.f, 40.f, 90.f)));
	REQUIRE(Vector3::isNearlyEqual(b/c, Vector3(.1f, .1f, .1f)));

	REQUIRE(isNearlyEqual(b.dotProduct(c), 10+40+90));
	REQUIRE(isNearlyEqual(b.length(), sqrt(1.f+4.f+9.f)));
	REQUIRE(isNearlyEqual(b.length2(), 1.f+4.f+9.f));
	REQUIRE(Vector3::isNearlyEqual(b.getNormal(), Vector3(1.f / 3.74165738677f, 2.f / 3.74165738677f, 3.f / 3.74165738677f)));

	Vector3 x(1,0,0);
	Vector3 y(0,1,0);
	Vector3 z = Vector3::crossProduct(x,y);
	REQUIRE(Vector3::isNearlyEqual(z, Vector3(0,0,1)));

	Vector4 w(1,2,3,4);
	REQUIRE((w.x == 1, w.y == 2, y.z == 3, w.w == 4));
}