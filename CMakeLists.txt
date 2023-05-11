cmake_minimum_required(VERSION 3.19)
project(eacham)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

# set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message("Your C++ compiler supports these C++ features:")
foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
  message("${i}")
endforeach()

find_package(OpenCV 4.7 REQUIRED)
message("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")

find_package(Pangolin REQUIRED)
message("Pangolin_INCLUDE_DIRS = ${Pangolin_INCLUDE_DIRECTORIES}")
message("Pangolin_LIBRARIES = ${Pangolin_LIBRARIES}")

find_package(Eigen3 REQUIRED)
message("EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}")

find_package(PCL REQUIRED)
message("PCL_LIBRARIES = ${PCL_LIBRARIES}")

find_package(GTSAM REQUIRED)
find_package(realsense2 QUIET)

if (${realsense2_FOUND})
    set(REALSENSE_FOUND 1)
    add_definitions(-DREALSENSE_FOUND)
endif()

set(SOURCE
    src/modules/map/Map.cpp

    # src/modules/motion_estimator/MotionEstimatorBase.cpp
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
target_link_libraries(main 
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
    gtsam
    metis-gtsam
)
set_property(TARGET main PROPERTY CXX_STANDARD 20)

if (${REALSENSE_FOUND})
    target_link_libraries(main
        realsense2::realsense2
    )
endif()

message(STATUS "REALSENSE_FOUND: " ${REALSENSE_FOUND})