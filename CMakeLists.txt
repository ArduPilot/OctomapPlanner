cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions(-DSRC_DIR=\"${CMAKE_SOURCE_DIR}\")

## Stereo image processing library
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
link_directories(${OpenCV_LIBS_DIR})

add_library(StereoMatcher SHARED src/stereo_matcher.cpp)
target_link_libraries(StereoMatcher ${OpenCV_LIBS})
##

## Octomap Library
find_package(octomap REQUIRED)

add_library(OctomapServer SHARED src/OctomapServer.cpp)
add_definitions(-DOCTOMAP_NODEBUGOUT)
target_link_libraries(OctomapServer ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
##

## Mavlink communication library
find_package(mavlink REQUIRED)
include_directories(${mavlink_INCLUDE_DIRS})

add_library(MavlinkComm SHARED src/mavlink_comm.cpp)
target_compile_options(MavlinkComm PRIVATE "-w")
##

## Planner Library
find_package(OMPL REQUIRED)
find_package(fcl REQUIRED)

add_library(Planner SHARED src/Planner.cpp)
include_directories(${OMPL_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS})
message("Libs: ${OCTOMAP_LIBRARIES}")
target_link_libraries(Planner fcl ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
##

## Gazebo visualizations
find_package(gazebo REQUIRED)
if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
    message("Not going to build visualizations")
    message(FATAL_ERROR "You need at least Gazebo 8.0. Your version: ${GAZEBO_VERSION}")
else()
    message("Gazebo version: ${GAZEBO_VERSION}")
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(GazeboVis SHARED src/gazebo_visualization.cpp)
target_link_libraries(GazeboVis ${GAZEBO_LIBRARIES})
##

## ArduPlanner library
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(ArduPlanner SHARED src/arduplanner.cpp)
target_link_libraries(ArduPlanner ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} pthread StereoMatcher OctomapServer MavlinkComm GazeboVis Planner)
##

## Main node
add_executable(main_node src/main_node.cpp)
target_link_libraries(main_node ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ArduPlanner)
target_compile_options(main_node PRIVATE "-w")
##