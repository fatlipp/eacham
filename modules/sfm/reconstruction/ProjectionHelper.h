#pragma once

#include "base/tools/Tools3d.h"
#include "sfm/data/Map.h"
#include "sfm/data/Types.h"
#include "sfm/data/Graph.h"

#include <opencv4/opencv2/opencv.hpp>
#include <Eigen/Geometry>

namespace eacham
{

Eigen::Matrix4d ConvertToTransformInv(const cv::Mat& R, const cv::Mat& t);

Eigen::Matrix4d ConvertToTransformInvManual(const cv::Mat& R, const cv::Mat& t);

Eigen::Vector3d ViewDirection(const Eigen::Matrix4d& matrix);

float CalcReprojectionError(const cv::Point2f& point2d, const cv::Point3f& point3d, const cv::Mat& K);

} // namespace eacham
