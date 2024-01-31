find_package(OpenCV REQUIRED)

add_library(lightglue_feature
    feature/FeatureExtractorLightglue.cpp
    feature/FeatureExtractorLightglue.h
    feature/FeatureMatcherLightglue.cpp
    feature/FeatureMatcherLightglue.h
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

