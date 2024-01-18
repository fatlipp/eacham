cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

# set(CMAKE_C_STANDARD    17)
set(CMAKE_CUDA_STANDARD 17)
project(cmake_and_cuda CXX CUDA)

find_package(CUDA 12.2 REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
message(STATUS "CUDA libs: ${CUDA_LIBRARIES}")

# lib
add_library(cuda_test_lib STATIC
  src/particle.cu
  src/v3.cu
)
set_target_properties(cuda_test_lib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(cuda_test_lib ${CUDA_LIBRARIES})
set_target_properties(cuda_test_lib PROPERTIES CUDA_ARCHITECTURES "86")

# exe
add_executable(cuda_test_app 
    src/main.cu
)
target_compile_features(cuda_test_app PUBLIC cxx_std_17)
set_target_properties(cuda_test_app PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(cuda_test_app 
  cuda_test_lib
  opencv::opencv)
set_target_properties(cuda_test_app PROPERTIES CUDA_ARCHITECTURES "86")