
#pragma once

#include "types/DataTypes.h"
#include "tools/Tools3d.h"

#include <opencv2/calib3d.hpp>

namespace eacham
{

static std::tuple<float, float> CalcReprojectionError(const cv::Mat &image, const std::vector<cv::Point3f> &pts3d1, 
        const std::vector<cv::Point2f> &pts2d2, 
        const cv::Mat &camMat, const cv::Mat &distCoeffs,
        const cv::Mat &R, const cv::Mat &t, 
        const float errorThreshold, std::vector<int> &inliers)
{
    if (pts3d1.size() < 2)
    {
        return {-1, -1};
    }
    inliers.clear();

    cv::Mat reprIm2;
    cv::cvtColor(image, reprIm2, cv::COLOR_BGR2RGB);

    float errMean = 0.0f;
    float minErr = 999999;
    float maxErr = -10000;

    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    std::vector<cv::Point2f> imagePointsReproj;
    cv::projectPoints(pts3d1, rvec, t, camMat, distCoeffs, imagePointsReproj);

    std::vector<float> reprojectionErrors;

    cv::Mat cameraMatOneDim = cv::Mat(1, 4, CV_32F);
    cameraMatOneDim.at<float>(0, 0) = camMat.at<float>(0, 0);
    cameraMatOneDim.at<float>(0, 1) = camMat.at<float>(1, 1);
    cameraMatOneDim.at<float>(0, 2) = camMat.at<float>(0, 2);
    cameraMatOneDim.at<float>(0, 3) = camMat.at<float>(1, 2);

    for (int i = 0; i < pts3d1.size(); ++i)
    {
        const int id = i;//inliers[i];

        auto pt = pts3d1.at(id);
        auto pp1 = tools::transformPointD(pt, R, t);
        const auto pp = tools::project3dPoint(pp1, cameraMatOneDim);

        // real points
        cv::circle(reprIm2, pts2d2.at(id), 6, {0, 255, 0}, 3);
        cv::circle(reprIm2, pp, 4, {0, 255, 255}, 3);
        cv::line(reprIm2, pts2d2.at(id), pp, {0, 255, 255});

        // reprojected
        cv::circle(reprIm2, imagePointsReproj.at(id), 2, {255, 0, 255}, 3);
        cv::line(reprIm2, pts2d2.at(id), imagePointsReproj.at(id), {255, 0, 0});

        const float err1 = std::sqrt(std::pow(pts2d2.at(id).x - pp.x, 2) + std::pow(pts2d2.at(id).y - pp.y, 2));

        if (err1 < minErr)
        {
            minErr = err1;
        }
        if (maxErr < err1)
        {
            maxErr = err1;
        }

        if (err1 < errorThreshold)
        {
            reprojectionErrors.push_back(err1);
            inliers.push_back(i);

            errMean += err1;
        }
    }

    float errVar = 0;
    
    if (reprojectionErrors.size() > 0)
    {
        std::sort(reprojectionErrors.begin(), reprojectionErrors.end());

        float median = reprojectionErrors[reprojectionErrors.size() / 2];

        errMean = errMean / pts3d1.size();

        cv::imshow("ME: ProjectedPoints", reprIm2);

        float errSum = 0;

        for (int i = 0; i < reprojectionErrors.size(); ++i)
        {
            errSum += (reprojectionErrors[i] - errMean) * (reprojectionErrors[i] - errMean);
        }

        if (reprojectionErrors.size() > 0)
        {
            errVar = errSum / (reprojectionErrors.size() - 1);
        }

        std::cout << "===========================================" << std::endl;
        std::cout << "mean repr err: " << errMean << ", variance: " << errVar << std::endl;
        std::cout << "median repr err: " << median << std::endl;
        std::cout << "repr minErr: " << minErr << std::endl;
        std::cout << "repr maxErr: " << maxErr << std::endl;
        std::cout << "===========================================" << std::endl;
    }
    else
    {
        std::cout << "===========================================" << std::endl;
        std::cout << "mean repr err: " << errMean << ", variance: " << errVar << std::endl;
        std::cout << "NO INLIERS" << std::endl;
        std::cout << "NO INLIERS" << std::endl;
        std::cout << "NO INLIERS" << std::endl;
        std::cout << "===========================================" << std::endl;
    }

    return {errMean, errVar};
}

}