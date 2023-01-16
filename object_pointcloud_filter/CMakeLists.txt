cmake_minimum_required(VERSION 3.0)
project(object_pointcloud_filter)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  voxblox_ros

)
find_package(ompl REQUIRED)



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
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS
      roscpp
      roslib
   DEPENDS
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/FrontEndOMPLRRTStar.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test
  src/testFrontEnd.cpp
  src/FrontEndOMPLRRTStar.cpp)
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)
