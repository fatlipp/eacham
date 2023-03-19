#pragma once

#include "MotionEstimatorBase.h"
#include "odometry/features/FeatureExtractor.h"
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
            if (m[0].distance < 0.6f * m[1].distance)
            {
                pts1.push_back(m[0].queryIdx);
                pts2.push_back(m[0].trainIdx);
            }
        }

        return { pts1, pts2 };
    }

    void MotionEstimatorBase::CalcReprojectionError(const cv::Mat &image, const std::vector<cv::Point3f> &pts3d1, 
            const std::vector<cv::Point2f> &pts2d2, const cv::Mat &R, const cv::Mat &t)
    {
        cv::Mat reprIm2;
        cv::cvtColor(image, reprIm2, cv::COLOR_BGR2RGB);

        float err = 0.0f;
        float minErr = 999999;
        float maxErr = -10000;

        cv::Mat rvec;
        cv::Rodrigues(R, rvec);

        std::vector<cv::Point2f> imagePointsReproj;
        cv::projectPoints(pts3d1, rvec, t, this->cameraMat, this->distCoeffs, imagePointsReproj);

        for (int i = 0; i < pts3d1.size(); ++i)
        {
            const int id = i;//inliers[i];

            auto pt = pts3d1.at(id);
            auto pp1 = transformPointD(pt, R, t);
            const auto pp = project3dPoint(pp1, this->cameraMatOneDim);

            // real points
            cv::circle(reprIm2, pts2d2.at(id), 6, {0, 255, 0}, 3);
            cv::circle(reprIm2, pp, 4, {0, 255, 255}, 3);
            cv::line(reprIm2, pts2d2.at(id), pp, {0, 255, 255});

            // reprojected
            cv::circle(reprIm2, imagePointsReproj.at(id), 2, {255, 0, 255}, 3);
            cv::line(reprIm2, pts2d2.at(id), imagePointsReproj.at(id), {255, 0, 0});

            const float err1 = std::pow(pts2d2.at(id).x - pp.x, 2) + std::pow(pts2d2.at(id).y - pp.y, 2);

            err += err1;
            if (err1 < minErr)
            {
                minErr = err1;
            }
            if (maxErr < err1)
            {
                maxErr = err1;
            }
        }
                
        err = err / pts3d1.size();
        err = std::sqrt(err);

        cv::imshow("ME: ProjectedPoints", reprIm2);

        std::cout << "===========================================" << std::endl;
        std::cout << "repr err: " << err << std::endl;
        std::cout << "repr minErr: " << std::sqrt(minErr) << std::endl;
        std::cout << "repr maxErr: " << std::sqrt(maxErr) << std::endl;
        std::cout << "===========================================" << std::endl;
    }
}