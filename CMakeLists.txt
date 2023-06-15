cmake_minimum_required(VERSION 3.19)
project(eacham)


set(CMAKE_TOOLCHAIN_FILE conan_toolchain.cmake)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(vtk REQUIRED
)

find_package(Pangolin REQUIRED)
find_package(realsense2 QUIET)

message(STATUS "VTK_LIBRARIES = ${VTK_LIBRARIES}")

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

message(STATUS "realsense2_FOUND: ${realsense2_FOUND}")
message(STATUS "OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
message(STATUS "GLEW_INCLUDE_DIR: ${GLEW_INCLUDE_DIR}")

get_target_property(OUT opencv_core INTERFACE_LINK_LIBRARIES)
message(STATUS "OUT: ${OUT}")

target_link_libraries(main PRIVATE
    gtsam::gtsam
    opencv::opencv
    pcl::pcl
    pangolin::pangolin
)
# target_link_libraries(main  "-Wl,-rpath-link ${PROJECT_SOURCE_DIR}/build/libs")

if (${REALSENSE_FOUND})
    target_link_libraries(main PRIVATE
        realsense2::realsense2
    )
endif()

message(STATUS "REALSENSE_FOUND: " ${REALSENSE_FOUND})

# add_custom_command(TARGET main POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_directory
#         "${PROJECT_SOURCE_DIR}/libs"
#         $<TARGET_FILE_DIR:main>)