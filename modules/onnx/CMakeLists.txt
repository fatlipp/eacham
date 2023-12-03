set(ONNX_LIBS 
    ${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/lib/libonnxruntime.so
)

set(ONNX_INCLUDE 
    ${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/include
)

add_subdirectory(lightglue)