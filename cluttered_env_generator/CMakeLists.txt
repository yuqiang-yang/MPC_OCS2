cmake_minimum_required(VERSION 3.0)
project(cluttered_env_generator)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -Wno-dev")
# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  include
  # "/home/vision/yq_ws/my_mpc/src/FIESTA/include"
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include  ${catkin_INCLUDE_DIRS}
   CATKIN_DEPENDS
      roscpp
      roslib
      std_msgs
      geometry_msgs
      pcl_conversions
   DEPENDS
)


###########
## Build ##
###########

add_executable(${PROJECT_NAME}
  src/ClutteredEnvGenerator.cpp
  src/ClutteredEnvGeneratorNode.cpp
  # /home/vision/yq_ws/my_mpc/src/FIESTA/src/timing.cc
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
