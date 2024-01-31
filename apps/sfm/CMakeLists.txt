set(CMAKE_CXX_STANDARD 20)

message(ERROR "CMAKE_CXX_COMPILER_VERSION = ${CMAKE_CXX_COMPILER_VERSION}")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

set(SOURCE
    main.cpp
    view/Render.cpp
)

# executable
set(ExeName "sfm")

add_executable(${ExeName})

target_sources(${ExeName}
    PRIVATE
        main.cpp
        view/Render.cpp
)
target_include_directories(${ExeName}
    PRIVATE
        src
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ROOT_FOLDER}/modules
        ${ROOT_FOLDER}/thirdparty
)
target_link_libraries(${ExeName} PRIVATE
    eacham_base
    eacham_sfm_lib
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
target_compile_options(${ExeName} PRIVATE -Wno-conversion -Wno-narrowing)

# transform changer
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