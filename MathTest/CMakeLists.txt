cmake_minimum_required(VERSION 3.7)
project(MathTest)

set(CMAKE_MODULE_PATH ../CMake)

include_directories(../Public)
include_directories(../External)

set(Source MathTest.cpp)
set(Headers ../Public/VectorOps.h ../Public/EqualOps.h ../Public/PairwiseOps.h ../Public/Vector3.h ../Public/Vector4.h ../Public/OCMath.h ../Public/Quaternion.h ../Public/Scaler3.h ../Public/Float3.h ../Public/Scaler4.h ../Public/Float4.h ../Public/TVector3.h ../Public/TVector4.h ../Public/Transform.h)

add_executable(MathTest ${Source} ${Headers})
