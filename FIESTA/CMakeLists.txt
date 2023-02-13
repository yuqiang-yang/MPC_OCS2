cmake_minimum_required(VERSION 3.0)
project(fiesta)
# if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
#      set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
# endif()
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        std_srvs
        visualization_msgs
        tf
        cv_bridge
        std_msgs
        message_generation
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
add_service_files(
        FILES
        filename.srv
      )
generate_messages(
        DEPENDENCIES
        std_msgs
)
catkin_package(
  INCLUDE_DIRS include ${PCL_INCLUDE_DIRS}
  LIBRARIES
    fiestalib
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)


include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror


set(CMAKE_BUILD_TYPE "RELEASE")
#add_executable(test_esdf_map
#        include/ESDFMap.h
#        src/ESDFMap.cpp
#        include/timing.h
#        src/timing.cc
#        test/test_ESDF_Map.cpp
#        )
#target_link_libraries(test_esdf_map
#        ${catkin_LIBRARIES}
#        )
#FIND_PACKAGE( OpenMP REQUIRED)
#if(OPENMP_FOUND)
#message("OPENMP FOUND")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

add_library(fiestalib
        include/parameters.h
        src/parameters.cpp
        include/timing.h
        src/timing.cc
        include/raycast.h
        src/raycast.cpp
        include/ESDFMap.h
        src/ESDFMap.cpp
        include/Fiesta.h
        test/test_fiesta.cpp)
target_link_libraries(fiestalib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        )
add_executable(test_fiesta
        include/parameters.h
        src/parameters.cpp
        include/timing.h
        src/timing.cc
        include/raycast.h
        src/raycast.cpp
        include/ESDFMap.h
        src/ESDFMap.cpp
        include/Fiesta.h
        test/test_fiesta.cpp
        )
target_link_libraries(test_fiesta
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        )


install(TARGETS fiestalib
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

