cmake_minimum_required(VERSION 3.7)
project(Sandbox)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)

add_subdirectory(MathTest)
add_subdirectory(RenderTest)
add_subdirectory(ConstraintTest)
add_subdirectory(GJKTest)