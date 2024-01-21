cmake_minimum_required(VERSION 3.19)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_STANDARD 20)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/base)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cuda)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/onnx)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/visualization)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sfm)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/physics)

# include(${CMAKE_CURRENT_SOURCE_DIR}/data_source)
# include(${CMAKE_CURRENT_SOURCE_DIR}/map)
# include(${CMAKE_CURRENT_SOURCE_DIR}/odometry)