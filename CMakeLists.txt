cmake_minimum_required(VERSION 3.12)

# Define the project
project(testing_robot_library
        DESCRIPTION "Executables for numerical simulation of RobotLibrary classes."
        LANGUAGES CXX
        VERSION 1.0)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

####################################################################################################
#                                           DEPENDENCIES                                           #
####################################################################################################

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(RobotLibrary REQUIRED)

# Include Eigen headers
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${RobotLibrary_INCLUDE_DIRS})

####################################################################################################
#                                               MODEL                                              #
####################################################################################################

add_executable(load_model src/model/load_model.cpp)
target_link_libraries(load_model PRIVATE Eigen3::Eigen RobotLibrary::RobotLibrary)

####################################################################################################
#                                            TRAJECTORY                                            #
####################################################################################################

add_executable(spline src/trajectory/spline.cpp)
target_link_libraries(spline PRIVATE RobotLibrary::RobotLibrary)

add_executable(trapezoidal_velocity src/trajectory/trapezoidal_velocity.cpp)
target_link_libraries(trapezoidal_velocity PRIVATE RobotLibrary::RobotLibrary)

####################################################################################################
#                                              CONTROL                                             #
####################################################################################################

add_executable(joint_control src/control/joint_control.cpp)
target_link_libraries(joint_control PRIVATE RobotLibrary::RobotLibrary)

add_executable(cartesian_control src/control/cartesian_control.cpp)
target_link_libraries(cartesian_control PRIVATE RobotLibrary::RobotLibrary)
