set(CMAKE_CXX_STANDARD 20)

message(ERROR "CMAKE_CXX_COMPILER_VERSION = ${CMAKE_CXX_COMPILER_VERSION}")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
# find_package(flann REQUIRED)

set(SOURCE

reconstruction/BundleAdjuster.cpp
    # reconstruction/ReconstructionManager.cpp
    # reconstruction/ProjectionHelper.cpp
    # reconstruction/Triangulator.cpp
    # view/Render.cpp
    obj_viewer.cpp
)

set(PROJ_NAME "eacham_sfm")

add_executable(${PROJ_NAME})

target_sources(${PROJ_NAME}
    PRIVATE
        ${SOURCE}
)
target_include_directories(${PROJ_NAME}
    PRIVATE
        src
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ROOT_FOLDER}/thirdparty
)

target_link_libraries(${PROJ_NAME} PRIVATE
    eacham_base
    opencv::opencv
    gtsam
    lightglue_feature
    tbb
    PCL::PCL
    Eigen3::Eigen
    # visualization
    pangolin::pangolin
    ${VTK_LIBRARIES}
)

# target_compile_options(${PROJ_NAME} PRIVATE -Werror -Wall -Wextra)
target_compile_options(${PROJ_NAME} PRIVATE -Wno-conversion -Wno-narrowing)


add_executable(TransformToNerf)
target_sources(TransformToNerf
    PRIVATE
        TransformToNerf.cpp
)
target_include_directories(TransformToNerf
    PRIVATE
        src
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ROOT_FOLDER}/thirdparty
)
target_link_libraries(TransformToNerf PRIVATE
    Eigen3::Eigen
)