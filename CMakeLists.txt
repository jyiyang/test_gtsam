cmake_minimum_required(VERSION 2.6.0)
project(test_gtsam)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})
include_directories( "/usr/include/eigen3" )
include_directories(include)

add_executable(LocalizationExample src/LocalizationExample.cpp)
add_executable(testFusion src/testFusion.cpp src/PoseFusion.cpp)

target_link_libraries(
    LocalizationExample
    gtsam
)

target_link_libraries(
    testFusion
    gtsam
)
