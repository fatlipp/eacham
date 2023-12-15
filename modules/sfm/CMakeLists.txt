set(CMAKE_CXX_STANDARD 20)

message(ERROR "CMAKE_CXX_COMPILER_VERSION = ${CMAKE_CXX_COMPILER_VERSION}")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(flann REQUIRED)

set(SOURCE
    ba/MotionBA.cpp
    reconstruction/EpipolarGeometry.cpp
    reconstruction/ProjectionHelper.cpp
    reconstruction/Triangulator.cpp
    view/Render.cpp
    main.cpp
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
    PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJ_NAME} PRIVATE
    opencv::opencv
    pangolin::pangolin
    gtsam
    lightglue_feature
    eacham_base
    # visualization
    tbb
)

# target_compile_options(${PROJ_NAME} PRIVATE -Werror -Wall -Wextra)
target_compile_options(${PROJ_NAME} PRIVATE -Wno-conversion -Wno-narrowing)