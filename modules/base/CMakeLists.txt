set(MODULE_NAME "eacham_base")

find_package(OpenCV REQUIRED)

add_library(${MODULE_NAME}
    features/FeatureExtractorSift.cpp
    features/FeatureMatcherFlann.cpp
    performance/BlockTimer.cpp
)
target_link_libraries(${MODULE_NAME}
    PRIVATE
    opencv::opencv
)
# target_include_directories(${MODULE_NAME} 
#     PRIVATE
        
# )