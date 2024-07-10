cmake_minimum_required(VERSION 3.12)
project(testing_robot_library)

# Set the default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
endif()

# Print the build type
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package(Eigen3 REQUIRED)
find_package(RobotLibrary REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(urdf_test src/urdf_test.cpp)
target_link_libraries(urdf_test PRIVATE RobotLibrary::RobotLibrary)

add_executable(spline_test src/spline_test.cpp)
target_link_libraries(spline_test PRIVATE RobotLibrary::RobotLibrary)

add_executable(trapezoidal_test src/trapezoidal_test.cpp)
target_link_libraries(trapezoidal_test PRIVATE RobotLibrary::RobotLibrary)
