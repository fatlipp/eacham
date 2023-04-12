#include "MotionEstimatorBase.h"
#include "tools/Tools3d.h"

#include <opencv2/calib3d.hpp>

namespace eacham
{
    std::tuple<std::vector<int>, std::vector<int>> MotionEstimatorBase::FindMatches(const Frame& frame1, const Frame& frame2)
    {
        const cv::Mat descriptor1 = frame1.GetDescriptors();
        const cv::Mat descriptor2 = frame2.GetDescriptors();

        std::vector<std::vector<cv::DMatch>> matches;
        this->mather->knnMatch(descriptor1, descriptor2, matches, 2);

        std::vector<int> pts1; 
        std::vector<int> pts2; 

        for (const auto& m : matches)
        {
            if (m[0].distance < 0.65f * m[1].distance 
                && tools::GetDistance(frame1.GetPoint3d(m[0].queryIdx), frame2.GetPoint3d(m[0].trainIdx)) < 1.0f)
            {
                pts1.push_back(m[0].queryIdx);
                pts2.push_back(m[0].trainIdx);
            }
        }

        return { pts1, pts2 };
    }

}