cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(mavlink REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${mavlink_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OpenCV_LIBS_DIR})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_definitions(-DSRC_DIR=\"${CMAKE_SOURCE_DIR}\")

add_library(StereoMatcher SHARED stereo_matcher.cpp)
target_link_libraries(StereoMatcher ${OpenCV_LIBS})

find_package(PCL REQUIRED COMPONENTS common io visualization sample_consensus segmentation filters)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

add_library(OctomapServer SHARED OctomapServer.cpp)
# target_compile_options(OctomapServer PRIVATE "-Wall")
target_link_libraries(OctomapServer ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_library(MavlinkComm SHARED mavlink_comm.cpp)
target_compile_options(MavlinkComm PRIVATE "-w")

add_executable(main_node main.cpp)
target_link_libraries(main_node ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} pthread StereoMatcher OctomapServer MavlinkComm)

# add_executable(mavlink_test mavlink_test.c)