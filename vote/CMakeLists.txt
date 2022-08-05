cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(aggregate)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(Qt5_DIR /usr/local/Cellar/qt@5/5.15.5_1/lib/cmake/Qt5)
endif()
find_package(PCL)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (aggregate aggregate.cpp)
target_link_libraries (aggregate ${PCL_LIBRARIES})
