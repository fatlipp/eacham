#include "sfm/reconstruction/ProjectionHelper.h"

#include "base/tools/Tools3d.h"
#include <opencv4/opencv2/calib3d.hpp>

#include <iostream>

namespace eacham
{

Eigen::Matrix4d ConvertToTransformInv(const cv::Mat& R, const cv::Mat& t)
{
    // Non const to use ptr*
    cv::Mat Rinv = R;
    cv::Mat tinv = t;

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        R_Eigen(Rinv.ptr<double>(), Rinv.rows, Rinv.cols);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        T_Eigen(tinv.ptr<double>(), tinv.rows, tinv.cols);

    Eigen::Affine3d motion = Eigen::Affine3d::Identity();
    motion.linear() = R_Eigen;
    motion.translation() = T_Eigen;

    // from camera to world CS
    // return motion.inverse(Eigen::TransformTraits::Isometry).matrix(); // from camera to world CS
    return motion.matrix(); // from camera to world CS
}

Eigen::Matrix4d ConvertToTransformInvManual(const cv::Mat& R, const cv::Mat& t)
{
    // from camera to world CS
    cv::Mat Rinv = R.inv();
    cv::Mat tinv = -Rinv * t;

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        R_Eigen(Rinv.ptr<double>(), Rinv.rows, Rinv.cols);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
        T_Eigen(tinv.ptr<double>(), tinv.rows, tinv.cols);

    Eigen::Matrix4d matr = Eigen::Matrix4d::Identity();
    matr.template topLeftCorner<3, 3>() = R_Eigen;
    matr.template topRightCorner<3, 1>() = T_Eigen;
    // matr.template block<1,3>(3,0).setZero();
    // matr.coeffRef(3,3) = 1;
    return matr;
}

Eigen::Vector3d ViewDirection(const Eigen::Matrix4d& matrix)
{
  return matrix.block<1, 3>(2, 0);
}

float CalcReprojectionError(const cv::Point2f& point2d, const cv::Point3f& point3d, const cv::Mat& K)
{
    const auto manualProj = tools::project3dPoint(point3d, K);
    return std::sqrt(std::pow(point2d.x - manualProj.x, 2) + std::pow(point2d.y - manualProj.y, 2));
}

} // namespace eacham
