cmake_minimum_required(VERSION 3.19)
project(eacham-core)

set(PROJ_NAME "eacham_camera")

find_package(OpenCV REQUIRED)

set(SOURCE 
    ""
)


add_executable(${PROJ_NAME})

target_sources(${PROJ_NAME}
    PRIVATE
        ${SOURCE}
)
target_include_directories(${PROJ_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        thirdparty/nlohmann
)

target_link_libraries(${PROJ_NAME} PRIVATE
    opencv::opencv
)