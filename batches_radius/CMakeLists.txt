cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pc2chunks8)

find_package(PCL 1.8 REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (batches batches.cpp)
target_link_libraries (batches ${PCL_LIBRARIES})
