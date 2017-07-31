#ifndef OC_PHYSWORLD_H
#define OC_PHYSWORLD_H

#include <vector>
#include "RigidBody.h"

class PhysWorld
{
public:
	void Simulate(float DeltaTime);

private:
	std::vector<RigidBody> bodies;
};

#endif