cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(prediction)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(Qt5_DIR /usr/local/Cellar/qt@5/5.15.5_1/lib/cmake/Qt5)
endif()
find_package(PCL)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PDAL 1.0.0 REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})
link_directories(${PDAL_LIBRARY_DIRS})
add_definitions(${PDAL_DEFINITIONS})

include(pybind11.cmake)

add_executable (prediction prediction.cpp ../util/pointCloudFileReader.cpp ../util/offsetManager.cpp)
target_link_libraries (prediction ${PCL_LIBRARIES} ${PDAL_LIBRARIES} pybind11::embed)
