# eacham
{in dev.}

Visual SLAM

Implemented: 
* TUM and KITTI datasets
* Realsense (D435i) (hardcoded: RGBD mode, 640x480)

usage: ./main path-to-config/Config*.json

/config - config folder

Odometry:
* Optimization based (BA)
* PnP

Dependencies:
* GTSAM (Local map optimization and Odometry)
* OpenCV (Image reading, processing, feature extracting and matching)
* Eigen3 (Matrix operations)
* Pangolin (Visualization)
* nlohmann (JSON parser, included in thirdparty)
* librealsense2 (OPTIONAL)
