# eacham-sfm
{in dev.}

Usage:
./build/modules/sfm/eacham_sfm PATH_TO_IMAGES_FOLDER [optional: START_ID [END_ID]]

The result will be written to 'positions.txt' 
Output format: ((frame_id:uint, position:Mat4x4) X valid frames count)

Dependencies:
* GTSAM (optimization)
* OpenCV (Image reading, processing, feature extracting and matching)
* Eigen3 (Matrix operations)
* Pangolin (Visualization)
* nlohmann (JSON parser, included in thirdparty)
* LightGlue (ONNX, included in thirdparty)
* librealsense2 (optional)
* Conan 2.x.x (optinal)