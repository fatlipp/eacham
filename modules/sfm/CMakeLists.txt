set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

set(SOURCE
    reconstruction/BundleAdjuster.cpp
    reconstruction/ReconstructionManager.cpp
    reconstruction/ProjectionHelper.cpp
    reconstruction/Triangulator.cpp
)

# lib
set(LibName "eacham_sfm_lib")

add_library(${LibName})

target_sources(${LibName}
    PRIVATE
        ${SOURCE}
)
target_include_directories(${LibName}
    PRIVATE
        src
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ROOT_FOLDER}/thirdparty
)
target_link_libraries(${LibName} PRIVATE
    eacham_base
    lightglue_feature
    gtsam
    opencv::opencv
    Eigen3::Eigen
)