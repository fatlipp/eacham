#pragma once

#include "IMotionEstimator.h"

#include "types/DataTypes.h"

namespace eacham
{

class MotionEstimatorBase : public IMotionEstimator
{
public:
    MotionEstimatorBase(const cv::Mat &cameraMatInp, const cv::Mat &distCoeffsInp)
    {
        if (cameraMatInp.rows == 1)
        {
            cameraMat = cv::Mat::eye(3, 3, CV_32FC1);
            cameraMat.at<float>(0, 0) = cameraMatInp.at<float>(0, 0);
            cameraMat.at<float>(1, 1) = cameraMatInp.at<float>(0, 1);
            cameraMat.at<float>(0, 2) = cameraMatInp.at<float>(0, 2);
            cameraMat.at<float>(1, 2) = cameraMatInp.at<float>(0, 3);
            cameraMat.at<float>(2, 2) = 1.0f;
        }
        else
        {
            std::cerr << "MotionEstimatorBase() Wrong camera parameters\n";
        }

        this->distCoeffs = distCoeffsInp;
    }

protected:
    cv::Mat cameraMat;
    cv::Mat distCoeffs;

};

}
