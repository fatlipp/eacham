#pragma once

#include "IMotionEstimator.h"
#include "odometry/features/FeatureMatcherFactory.h"

#include "odometry/frame/Frame.h"
#include "types/DataTypes.h"

namespace eacham
{

class MotionEstimatorBase : public IMotionEstimator
{
public:
    MotionEstimatorBase(const FeatureExtractorType &featureExtractor)
    {
        this->mather = CreateFeatureMatcher(featureExtractor);
    }

public:
    std::tuple<std::vector<int>, std::vector<int>> FindMatches(const Frame& frame1, const Frame& frame2);

    void CalcReprojectionError(const cv::Mat &image, const std::vector<cv::Point3f> &pts3d1, const std::vector<cv::Point2f> &pts2d2,
        const cv::Mat &R, const cv::Mat &t);

protected:
    cv::Mat cameraMat;
    cv::Mat cameraMatOneDim;
    cv::Mat distCoeffs;

private:
    matcher_t mather;

};

}
