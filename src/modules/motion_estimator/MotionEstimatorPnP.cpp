#include "MotionEstimatorPnP.h"
#include "tools/Tools3d.h"

#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

#include "motion_estimator/MotionEstimatorTools.h"

namespace eacham
{

EstimationResult MotionEstimatorPnP::Estimate(const IFrameLight& frame1, const IFrame& frame2,
    const std::vector<std::pair<unsigned, unsigned>>& matches)
{
    std::vector<cv::Point3f> pts3d1;
    std::vector<cv::Point2f> pts2d2; 

    std::for_each(matches.begin(), matches.end(), [&](const auto& math){
        pts3d1.push_back(frame1.GetPointData(std::get<0>(math)).position3d);
        pts2d2.push_back(frame2.GetPointData(std::get<1>(math)).keypoint);
    });

    cv::Mat rvec = cv::Mat_<double>(3, 1);
    cv::Mat tvec = cv::Mat_<double>(3, 1);

    std::vector<int> inliersPnP;
    cv::solvePnPRansac(pts3d1, pts2d2, cameraMat, distCoeffs, rvec, tvec, false, 100, 4.0f, 0.99f, inliersPnP, cv::SOLVEPNP_EPNP);

    std::cout << "inliers (pnp): " << inliersPnP.size() << " (" << (inliersPnP.size() / static_cast<float>(matches.size())) << ")" << std::endl;

    Eigen::Affine3f motion = Eigen::Affine3f::Identity();

    if (inliersPnP.size() >= 10)
    {
        cv::Mat Rmat;
        cv::Rodrigues(rvec, Rmat);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> R_Eigen(Rmat.ptr<double>(), Rmat.rows, Rmat.cols);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> T_Eigen(tvec.ptr<double>(), tvec.rows, tvec.cols);

        motion.linear() = R_Eigen.cast<float>();
        motion.translation() = T_Eigen.cast<float>();
        motion = motion.inverse();

        EstimationResult result;
        result.frameIdPrev = frame1.GetId();
        result.frameIdCurrent = frame2.GetId();
        result.odometry = motion.matrix();

        for (const auto& inlier : inliersPnP)
        {
            const auto m1 = frame1.GetPointData(matches[inlier].first).id;
            const auto m2 = frame2.GetPointData(matches[inlier].second).id;
            result.matches.insert({m2, m1});
        }

        return result;
    }

    return { .frameIdPrev = 0, .frameIdCurrent = 0, .odometry = Eigen::Matrix4f::Identity() };
}

}