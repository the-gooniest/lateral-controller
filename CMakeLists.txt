cmake_minimum_required(VERSION 2.8.3)
project(lateral_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  bolt_msgs
)
find_package(cmake_modules REQUIRED)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE SOURCES
    include/*.h
    src/*.cpp
)

MESSAGE("${SOURCES}")

add_executable(controller ${SOURCES})
target_link_libraries(controller ${catkin_LIBRARIES})

