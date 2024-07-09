cmake_minimum_required(VERSION 3.12)
project(testing_robot_library)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(RobotLibrary REQUIRED)

add_executable(urdf_test src/urdf_test.cpp)

# Include directories for RobotLibrary
target_include_directories(urdf_test PRIVATE ${RobotLibrary_INCLUDE_DIRS})

# Link against the Model library from RobotLibrary
target_link_libraries(urdf_test PRIVATE RobotLibrary::Model RobotLibrary::Control)
