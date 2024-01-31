#include "sfm/reconstruction/ProjectionHelper.h"

#include "base/tools/Tools3d.h"

namespace eacham
{

Eigen::Matrix4d ConvertToTransform(const cv::Mat& R, const cv::Mat& t)
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

    // return motion.inverse(Eigen::TransformTraits::Isometry).matrix(); // from camera to world CS
    return motion.matrix();
}

Eigen::Vector3d ViewDirection(const Eigen::Matrix4d& matrix)
{
  return matrix.block<1, 3>(2, 0);
}

float CalcReprojectionError(const Eigen::Vector2d& point2d, 
    const Eigen::Vector3d& point3d, const cv::Mat& K)
{
    const auto manualProj = tools::Project3dPoint(point3d, K);
    return std::sqrt(std::pow(point2d.x() - manualProj.x(), 2) + 
        std::pow(point2d.y() - manualProj.y(), 2));
}

} // namespace eacham
