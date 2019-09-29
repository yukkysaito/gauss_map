cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gauss_map)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (gauss_map_sample src/gauss_map_sample.cpp)
target_link_libraries (gauss_map_sample ${PCL_LIBRARIES})