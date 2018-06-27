cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(mavlink REQUIRED)
find_package(OMPL REQUIRED)
find_package(fcl REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${mavlink_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
link_directories(${GAZEBO_LIBRARY_DIRS} ${OpenCV_LIBS_DIR})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_definitions(-DSRC_DIR=\"${CMAKE_SOURCE_DIR}\")

add_library(StereoMatcher SHARED src/stereo_matcher.cpp)
target_link_libraries(StereoMatcher ${OpenCV_LIBS})

find_package(PCL REQUIRED COMPONENTS common io visualization sample_consensus segmentation filters)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

add_library(OctomapServer SHARED src/OctomapServer.cpp)
# target_compile_options(OctomapServer PRIVATE "-Wall")
target_link_libraries(OctomapServer ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_library(MavlinkComm SHARED src/mavlink_comm.cpp)
target_compile_options(MavlinkComm PRIVATE "-w")

add_library(Planner SHARED src/Planner.cpp)
include_directories(${OMPL_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS})
target_link_libraries(Planner ${OCTOMAP_LIBRARIES} ${fcl_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(main_node src/main.cpp)
target_link_libraries(main_node ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} pthread StereoMatcher OctomapServer MavlinkComm)
target_compile_options(main_node PRIVATE "-w")
# add_executable(mavlink_test mavlink_test.c)