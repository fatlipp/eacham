#include "base/tools/Tools3d.h"
#include "sfm/data/Map.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <iostream>
#include <random>

namespace eacham
{
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = M_PI / 180.0;

struct EstimatorData
{
    Eigen::Matrix4d transform;
    Eigen::Vector2d point2d;
    cv::Mat K;
    
};

bool CheckTriangulationAngle(const Eigen::Matrix4d& cam1,
                            const Eigen::Matrix4d& cam2,
                            const Eigen::Vector3d& point3D,
                            const double angleLim = 1.5);

Eigen::Vector3d TriangulatePoint(const Eigen::Matrix4d& cam1Pos,
                                 const Eigen::Matrix4d& cam2Pos,
                                 const Eigen::Vector2d& point1,
                                 const Eigen::Vector2d& point2);

Eigen::Vector3d TriangulatePoint(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const cv::Mat& K,
    const Eigen::Matrix4d& transform1, const Eigen::Matrix4d& transform2);

Eigen::Vector3d TriangulatePoint(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const cv::Mat& K,
    const Eigen::Matrix4d& transform);

void TriangulateFrame(const unsigned frameId, std::shared_ptr<graph_t> graph, 
    std::shared_ptr<Map> map, const cv::Mat& K, const unsigned minObservers);

}