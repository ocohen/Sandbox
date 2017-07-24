cmake_minimum_required(VERSION 3.8)
project(core)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${CMAKE_SOURCE_DIR}/Private)
include_directories(${CMAKE_SOURCE_DIR}/Public)
include_directories(${CMAKE_SOURCE_DIR}/External)

set(Source )
set(Headers Public/NFloat.h public/Vector.h Public/Vector3.h Public/Vector4.h Public/OCMath.h Public/Quaternion.h Public/TinyRenderer.h)
set(Common ${Source} ${Headers})

set(UnitTestSource Private/MathTest.cpp)
set(PlaygroundSource Private/Playground.cpp)

#OpenGL
find_package(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})

#SDL
find_package(SDL2)
include_directories(${SDL2_INCLUDE_DIR})


add_executable(UnitTest ${Common} ${UnitTestSource})
add_executable(Playground ${Common} ${PlaygroundSource})
target_link_libraries(Playground ${OPENGL_LIBRARY} ${SDL_MAIN} ${SDL2_LIBRARY})