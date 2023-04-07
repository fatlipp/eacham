cmake_minimum_required(VERSION 3.19)
project(eacham)

set (CMAKE_CXX_STANDARD 17)

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

set(SOURCE
    src/modules/map/LocalMap.cpp

    src/modules/motion_estimator/MotionEstimatorBase.cpp
    src/modules/motion_estimator/MotionEstimatorPnP.cpp
    src/modules/motion_estimator/MotionEstimatorOpt.cpp
    
    src/modules/features/FeatureExtractor.cpp
    src/modules/frame/FrameCreatorStereo.cpp
    src/modules/frame/FrameCreatorRgbd.cpp
    
    src/modules/optimization/LocalFramesOptimizer.cpp
)

# add_library(eacham_lib INTERFACE)
# target_sources(eacham_lib
#     INTERFACE
#         ${HEADER}
# )
# target_include_directories(eacham_lib
#     PRIVATE
#         # where the library itself will look for its internal headers
#         ${HEADER}
#     PUBLIC
#         # where top-level project will look for the library's public headers
#         ${OpenCV_INCLUDE_DIRS}
# )
# target_link_libraries(eacham_lib ${OpenCV_LIBRARIES})


# // exe
add_executable(read_dataset)
target_sources(read_dataset
    PRIVATE
        ${SOURCE}
        src/main.cpp
)
target_include_directories(read_dataset
    PRIVATE
        src/modules
        src/visualization
    PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(read_dataset 
    ${OpenCV_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
    gtsam
    metis-gtsam
    )
