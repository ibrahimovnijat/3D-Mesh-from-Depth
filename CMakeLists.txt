cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)
project(3d_mesh_from_depth)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)
find_package(Eigen3 REQUIRED)

set(HEADERS 
    Eigen.h
    FreeImageHelper.h
    VirtualSensor.h
)

set(SOURCES
    main.cpp
    FreeImageHelper.cpp
)

link_directories(${FreeImage_LIBRARY_DIR})
add_executable(3d_mesh_from_depth ${HEADERS} ${SOURCES})
target_include_directories(3d_mesh_from_depth PUBLIC ${EIGEN3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR})
target_link_libraries(3d_mesh_from_depth general freeimage)
