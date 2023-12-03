cmake_minimum_required(VERSION 3.19)

project(eacham)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_TOOLCHAIN_FILE conan_toolchain.cmake)
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/modules)