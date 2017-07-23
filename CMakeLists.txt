cmake_minimum_required(VERSION 3.8)
project(core)

set(Source Private/MathTest.cpp)
set(Headers Public/NFloat.h Public/Vector3.h Public/Vector4.h Public/OCMath.h Public/Quaternion.h)

#add_library(core STATIC )
add_executable(core ${Source} ${Headers})

include_directories(Private)
include_directories(Public)
include_directories(External)