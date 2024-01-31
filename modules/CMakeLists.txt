cmake_minimum_required(VERSION 3.19)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_STANDARD 20)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/base)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/onnx)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sfm)