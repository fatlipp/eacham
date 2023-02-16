cmake_minimum_required(VERSION 3.16)
project(eacham)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4.2 REQUIRED)

message("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")

# // lib
# set(HEADER
#     src/modules
# )

set(SOURCE
    src/modules/odometry/VisualOdometry.cpp

    src/modules/odometry/features/FeatureExtractor.cpp
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
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(read_dataset ${OpenCV_LIBRARIES} )
