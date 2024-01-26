set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CUDA_STANDARD 20)
project(cmake_and_cuda CXX CUDA)
message(ERROR "CMAKE_CXX_COMPILER_VERSION = ${CMAKE_CXX_COMPILER_VERSION}")

find_package(CUDA 12.2 REQUIRED)
find_package(glm REQUIRED)

get_filename_component(CUDA_COMPILER_BIN "${CMAKE_CUDA_COMPILER}" DIRECTORY)
get_filename_component(CUDA_DIR "${CUDA_COMPILER_BIN}" DIRECTORY)
set(CUDA_INCLUDE "${CUDA_DIR}/include")

get_target_property(TCNN_INCLUDE_DIRECTORIES tiny-cuda-nn INCLUDE_DIRECTORIES)

message(STATUS "TCNN_INCLUDE_DIRECTORIES = ${TCNN_INCLUDE_DIRECTORIES}")

set(SOURCE
    main.cpp
    cuda/CudaHelper.cpp
    render/Render.cpp
    soft/CudaRigid.cu
    soft/SoftObjects2d.cpp
    soft/SoftObjects3d.cpp
    tools/BlockTimer.cpp
)

set(PROJ_NAME "eacham_ngp")

add_executable(${PROJ_NAME})

target_sources(${PROJ_NAME}
    PRIVATE
        ${SOURCE}
)
target_include_directories(${PROJ_NAME}
    PRIVATE
        src
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CUDA_INCLUDE}
        ${ROOT_FOLDER}/thirdparty
)
target_link_libraries(${PROJ_NAME} 
PRIVATE
    GL
    GLU
    glut
        ${CUDA_LIBRARIES} 
        tiny-cuda-nn
        glm::glm
    )

list(APPEND CUDA_NVCC_FLAGS "--extended-lambda")
target_compile_features(${PROJ_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJ_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(${PROJ_NAME} PROPERTIES CUDA_ARCHITECTURES "86")
target_compile_definitions(${PROJ_NAME} PUBLIC -DTCNN_MIN_GPU_ARCH=0 -DGL_GLEXT_PROTOTYPES=1)
target_compile_options(${PROJ_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)

