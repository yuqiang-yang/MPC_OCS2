cmake_minimum_required(VERSION 3.0)
project(graceful_mpc)

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

set(PACKAGE_DEPENDENCIES
    roslib
    tf
    geometry_msgs
    ocs2_comm_interfaces
    ocs2_core
    ocs2_robotic_tools
    ocs2_ddp
    kindr
    kindr_ros
    frontend_ompl
    fiesta
)

find_package(catkin REQUIRED COMPONENTS
    ${PACKAGE_DEPENDENCIES}
    message_generation
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  )

add_message_files(
  FILES
  PoseVelocityStamped.msg
  PoseVelocityTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    graceful_mpc
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
  DEPENDS
    Boost
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_explicit_instantiations
  src/ExplicitTemplateInstantiations.cpp)

add_dependencies(${PROJECT_NAME}_explicit_instantiations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_explicit_instantiations
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
  )

add_library(${PROJECT_NAME}
  src/SystemDynamics.cpp
  src/costs/QuadraticEndeffectorTrackingCost.cpp
  src/costs/PointsOnRobot.cpp
  src/costs/FrontOrientationCost.cpp
  src/costs/ManipulabilityCost.cpp
  src/costs/FiestaCost.cpp
  src/costs/StateCost.cpp

  src/OperatingPoint.cpp
  src/kinematics/KinematicsInterface.cpp
  src/kinematics/ur5/UR5Kinematics.cpp
  
)

add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_explicit_instantiations
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )



add_library(${PROJECT_NAME}_example
        src/example/GracefulMpcInterface.cpp
        )

add_dependencies(${PROJECT_NAME}_example
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_example
        ${PROJECT_NAME}
        ${PROJECT_NAME}_explicit_instantiations
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

add_executable(${PROJECT_NAME}_kinematic_simulation
        src/example/kinematic_simulation_node.cpp
        src/example/KinematicSimulation.cpp)

add_dependencies(${PROJECT_NAME}_kinematic_simulation
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_kinematic_simulation
        ${PROJECT_NAME}
        ${PROJECT_NAME}_explicit_instantiations
        ${PROJECT_NAME}_example
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
