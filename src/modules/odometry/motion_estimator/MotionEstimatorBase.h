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
    std::tuple<std::vector<PointData>, std::vector<PointData>> Match(const Frame& frame1, Frame& frame2)
    {
        const cv::Mat descriptor1 = frame1.GetDescriptors();
        const cv::Mat descriptor2 = frame2.GetDescriptors();

        std::vector<std::vector<cv::DMatch>> matches;
        this->mather->knnMatch(descriptor1, descriptor2, matches, 2);

        std::vector<PointData> pts1; 
        std::vector<PointData> pts2; 

        for (const auto& m : matches)
        {
            if (m[0].distance < 0.7f * m[1].distance)
            {
                pts1.push_back(frame1.GetPointData(m[0].queryIdx));
                pts2.push_back(frame2.GetPointData(m[0].trainIdx));
            }
        }

        return { pts1, pts2 };
    }

private:
    matcher_t mather;

};

}
