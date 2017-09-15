cmake_minimum_required(VERSION 3.7)
project(GJKTest)

include_directories(../Public)
include_directories(../External)

set(GJKSource GJKTest.cpp)
set(GJKUnitTestSource GJKUnitTest.cpp)
set(GJKContactSource GJKContactTest.cpp)
set(Headers ../Public/MassProperties.h ../Public/ShapeRenderer.h ../Public/RigidActor.h ../Public/PhysWorld.h ../Public/RigidBody.h ../Public/Shape.h ../Public/Constraint.h ../Public/OCMath.h ../Public/Float3.h ../Public/VectorOps.h ../Public/EqualOps.h ../Public/PairwiseOps.h ../Public/Vector3.h ../Public/OCMath.h ../Public/Renderer.h ../Public/Scaler3.h ../Public/Float3.h ../Public/TVector3.h ../Public/Transform.h ../Public/Quaternion.h ../Public/GJK.h)

#OpenGL
find_package(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})

#SDL
find_package(SDL2)
include_directories(${SDL2_INCLUDE_DIR})


add_executable(GJKTest ${GJKSource} ${Headers})
add_executable(GJKContactTest ${GJKContactSource} ${Headers})
add_executable(GJKUnitTests ${GJKUnitTestSource} ${Headers})
target_link_libraries(GJKTest ${OPENGL_LIBRARY} ${SDL_MAIN} ${SDL2_LIBRARY})
target_link_libraries(GJKContactTest ${OPENGL_LIBRARY} ${SDL_MAIN} ${SDL2_LIBRARY})
