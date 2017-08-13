cmake_minimum_required(VERSION 3.7)
project(RenderTest)

set(CMAKE_MODULE_PATH../CMake)

include_directories(../Public)
include_directories(../External)

set(Source RenderTest.cpp)
set(Headers ../Public/VectorOps.h ../Public/EqualOps.h ../Public/PairwiseOps.h ../Public/Vector3.h ../Public/OCMath.h ../Public/Renderer.h ../Public/Scaler3.h ../Public/Float3.h ../Public/TVector3.h ../Public/Quaternion.h ../Public/Transform.h)

#OpenGL
find_package(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})

#SDL
find_package(SDL2)
include_directories(${SDL2_INCLUDE_DIR})


add_executable(RenderTest ${Source} ${Headers})
target_link_libraries(RenderTest ${OPENGL_LIBRARY} ${SDL_MAIN} ${SDL2_LIBRARY})
