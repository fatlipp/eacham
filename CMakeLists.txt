cmake_minimum_required(VERSION 3.19)

project(eacham)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)
set(ROOT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/modules)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps)