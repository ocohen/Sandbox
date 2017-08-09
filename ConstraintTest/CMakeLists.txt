cmake_minimum_required(VERSION 3.7)
project(Sandbox)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../CMake)

include_directories(${CMAKE_SOURCE_DIR}/../Public)
include_directories(${CMAKE_SOURCE_DIR}/../External)

set(ConstraintSource ConstraintTest.cpp)
set(PhysTestSource PhysUnitTest.cpp)
set(Headers ../Public/ShapeRenderer.h ../Public/RigidActor.h ../Public/PhysWorld.h ../Public/RigidBody.h ../Public/Shape.h ../Public/Constraint.h ../Public/OCMath.h ../Public/Float3.h ../Public/VectorOps.h ../Public/EqualOps.h ../Public/PairwiseOps.h ../Public/Vector3.h ../Public/OCMath.h ../Public/Renderer.h ../Public/Scaler3.h ../Public/Float3.h ../Public/TVector3.h ../Public/Transform.h ../Public/Quaternion.h)

#OpenGL
find_package(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})

#SDL
find_package(SDL2)
include_directories(${SDL2_INCLUDE_DIR})


add_executable(ConstraintTest ${ConstraintSource} ${Headers})
add_executable(PhysUnitTests ${PhysTestSource} ${Headers})
target_link_libraries(ConstraintTest ${OPENGL_LIBRARY} ${SDL_MAIN} ${SDL2_LIBRARY})
