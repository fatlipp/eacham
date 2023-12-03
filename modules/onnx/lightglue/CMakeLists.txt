find_package(OpenCV REQUIRED)

add_library(lightglue_feature
    core/FeatureExtractorLightglue.cpp
    core/FeatureExtractorLightglue.h
    core/FeatureMatcherLightglue.cpp
    core/FeatureMatcherLightglue.h
)
target_link_libraries(lightglue_feature
    opencv::opencv
    ${ONNX_LIBS}
)
target_include_directories(lightglue_feature 
    PUBLIC
        ${ONNX_INCLUDE}
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/example)

