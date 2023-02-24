#include "MotionEstimator.h"

namespace odometry
{

MotionEstimator::MotionEstimator()
{
}

MotionEstimator::~MotionEstimator()
{
}

Eigen::Matrix4f MotionEstimator::Estimate(const Frame& frame1, const Frame& frame2)
{
    const auto descriptor1 = frame1.GetDescriptors();
    const auto descriptor2 = frame2.GetDescriptors();

    const auto mather = std::make_unique<cv::BFMatcher>(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> matches;
    // this->mather->match(descriptor1, descriptor2, matches);
    mather->knnMatch(descriptor1, descriptor2, matches, 2);

    std::cout << "matches with prev size: " << matches.size() << std::endl;

    if (matches.size() < 50)
    {
        return Eigen::Matrix4f::Identity();
    }

    

    return Eigen::Matrix4f::Identity();
}

}