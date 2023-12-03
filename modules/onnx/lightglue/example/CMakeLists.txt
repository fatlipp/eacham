find_package(OpenCV REQUIRED)

add_executable(lightglue_seq 
    src/extract_match.cpp
)

target_link_libraries(lightglue_seq
    lightglue_feature
    opencv::opencv
)

add_executable(lightglue_e2e
    src/extract_end_2_end.cpp
)
target_link_libraries(lightglue_e2e
    lightglue_feature
    opencv::opencv
)
