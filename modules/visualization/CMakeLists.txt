# set(CMAKE_TOOLCHAIN_FILE conan_toolchain.cmake)
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

# find_package(OpenCV REQUIRED)
# find_package(Pangolin REQUIRED)

# set(SOURCE
#     render/Render.cpp
# )

# set(PROJ_NAME "visualization")

# add_library(${PROJ_NAME})

# target_sources(${PROJ_NAME}
#     PRIVATE
#         ${SOURCE}
# )
# target_include_directories(${PROJ_NAME}
#     PRIVATE
#         ${CMAKE_CURRENT_SOURCE_DIR}
#     PUBLIC
#         ${EIGEN3_INCLUDE_DIR}
#         ${OpenCV_INCLUDE_DIRS}
#         thirdparty/nlohmann
# )

# target_link_libraries(${PROJ_NAME} PRIVATE
#     opencv::opencv
#     pangolin::pangolin
# )

# vtk_module_autoinit(
#   TARGETS main
#   MODULES ${VTK_LIBRARIES}
# )