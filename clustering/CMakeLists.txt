cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(clustering)

find_package(PCL 1.11 REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (clustering clustering.cpp)
target_link_libraries (clustering ${PCL_LIBRARIES})
