cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# set(NEON_FLAGS "-DENABLE_NEON -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a15 -Ofast")
set(NEON_FLAGS "-O3 -march=native -mcpu=cortex-a15.cortex-a7 -mtune=cortex-a15.cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -fomit-frame-pointer -ffast-math")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE ${NEON_FLAGS})

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(mavlink REQUIRED)

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

add_library(optorimu SHARED loitor/src/optorimu.cpp)

add_library(optorusbcam SHARED loitor/src/optorusb.cpp loitor/src/optorcam.cpp)
target_link_libraries(optorusbcam optorimu)

add_executable(main_node src/main.cpp)
target_link_libraries(main_node ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} pthread StereoMatcher OctomapServer MavlinkComm optorusbcam usb-1.0)
target_compile_options(main_node PRIVATE "-w")
# add_executable(mavlink_test mavlink_test.c)
