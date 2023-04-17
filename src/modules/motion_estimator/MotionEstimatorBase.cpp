#include "MotionEstimatorBase.h"
#include "tools/Tools3d.h"

#include <opencv2/calib3d.hpp>

namespace eacham
{
    cv::Mat GetDescriptors(const IFrame& frame)
    {   
        const auto pointsData = frame.GetPointsData();

        cv::Mat result = cv::Mat(pointsData.size(), pointsData[0].descriptor.cols, pointsData[0].descriptor.type());

        for (int i = 0; i < pointsData.size(); ++i)
        {
            pointsData[i].descriptor.copyTo(result.row(i));
        }

        return result;
    }

    std::tuple<std::vector<int>, std::vector<int>> MotionEstimatorBase::FindMatches(const IFrame& frame1, const IFrame& frame2)
    {
        const cv::Mat descriptor1 = GetDescriptors(frame1);
        const cv::Mat descriptor2 = GetDescriptors(frame2);

        std::vector<std::vector<cv::DMatch>> matches;
        this->mather->knnMatch(descriptor1, descriptor2, matches, 2);

        std::vector<int> pts1; 
        std::vector<int> pts2;

        for (const auto& m : matches)
        {
            if (m[0].distance < 0.5f * m[1].distance)
            {
                pts1.push_back(m[0].queryIdx);
                pts2.push_back(m[0].trainIdx);
            }
        }

        std::cout << "descriptor1: " << descriptor1.rows 
                 << ", descriptor2: " << descriptor2.rows 
                 << ", matches: " << matches.size()
                 << ", good: " << pts1.size() << std::endl;

        return { pts1, pts2 };
    }

}