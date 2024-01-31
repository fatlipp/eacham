#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <Eigen/Geometry>

namespace eacham
{

Eigen::Matrix4d ConvertToTransform(const cv::Mat& R, const cv::Mat& t);

Eigen::Vector3d ViewDirection(const Eigen::Matrix4d& matrix);

float CalcReprojectionError(const Eigen::Vector2d& point2d, 
    const Eigen::Vector3d& point3d, const cv::Mat& K);

} // namespace eacham
