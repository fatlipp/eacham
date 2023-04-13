# eacham
{in dev.}

Visual SLAM

Ready for reading TUM and KITTI datasets (probably needs to correct config file and some paths in the source code)

usage: ./main path-to-config/ConfigTUM.json

/config - config folder

Odometry:
* Optimization based
* PnP

Dependencies:
* GTSAM (Local map optimization and Odometry)
* OpenCV (Image reading, processing, feature extracting and matching)
* Eigen3 (Matrix operations)
* Pangolin (Visualization)
* nlohmann (JSON parser, included in thirdparty)
