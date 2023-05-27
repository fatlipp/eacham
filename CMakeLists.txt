cmake_minimum_required(VERSION 3.19)
project(eacham)


set(CMAKE_TOOLCHAIN_FILE conan_toolchain.cmake)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

find_package(OpenCV 4.5.5 REQUIRED)
find_package(GTSAM 4.1.1 REQUIRED)
find_package(pcl 1.13.1 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(realsense2 QUIET)

if (${realsense2_FOUND})
    set(REALSENSE_FOUND 1)
    add_definitions(-DREALSENSE_FOUND)
endif()

set(SOURCE
    src/modules/map/Map.cpp
    src/modules/motion_estimator/MotionEstimatorPnP.cpp
    src/modules/motion_estimator/MotionEstimatorOpt.cpp
    src/modules/features/FeatureExtractor.cpp
    src/modules/frame/FrameCreatorStereo.cpp
    src/modules/frame/FrameCreatorRgbd.cpp
    src/modules/optimizer/MapOptimizerBA.cpp
    src/modules/visualization/Render.cpp
    src/modules/performance/BlockTimer.cpp
)

add_executable(main)
target_sources(main
    PRIVATE
        ${SOURCE}
        src/main.cpp
)
target_include_directories(main
    PRIVATE
        src/modules
        src/visualization
    PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        thirdparty/nlohmann
)

message(STATUS "Eigen3_INCLUDE_DIR: ${Eigen3_INCLUDE_DIR}")
message(STATUS "GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR}")

target_link_libraries(main
    gtsam::gtsam
    opencv_core
    opencv_features2d
    opencv_xfeatures2d
    opencv_highgui
    opencv_flann
    pcl::pcl
    ${Pangolin_LIBRARIES}
)

if (${REALSENSE_FOUND})
    target_link_libraries(main
        realsense2::realsense2
    )
endif()

message(STATUS "REALSENSE_FOUND: " ${REALSENSE_FOUND})

# add_custom_command(TARGET main POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_directory
#         "${PROJECT_SOURCE_DIR}/libs"
#         $<TARGET_FILE_DIR:main>)