cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(las2pcl)

find_package(PCL 1.10 REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PDAL 1.0.0 REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})
link_directories(${PDAL_LIBRARY_DIRS})
add_definitions(${PDAL_DEFINITIONS})

add_executable (las2pcl las2pcl.cpp ../util/pointCloudFileReader.cpp ../util/offsetManager.cpp)
target_link_libraries (las2pcl ${PCL_LIBRARIES} ${PDAL_LIBRARIES})
