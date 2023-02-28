cmake_minimum_required(VERSION 3.0)
project(frontend_ompl)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  fiesta

)
find_package(ompl REQUIRED)


include_directories("/home/vision/yq_ws/my_mpc/src/graceful_mpc/include")
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
   INCLUDE_DIRS include ${OMPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
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

add_executable(${PROJECT_NAME}_demo

  src/demoFrontEnd.cpp
)
target_link_libraries(${PROJECT_NAME}_demo
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

# add_executable(${PROJECT_NAME}_script
#   src/Urdf2RobCoGen_script.cpp)
# target_link_libraries(${PROJECT_NAME}_script
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY examples launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )