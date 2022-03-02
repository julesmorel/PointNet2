cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(filter)

find_package(PCL 1.11 REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (filter filter.cpp)
target_link_libraries (filter ${PCL_LIBRARIES})
