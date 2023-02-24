#pragma once

#include "types/DataTypes.h"

#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Core>

namespace odometry
{

class Frame
{
public:
    Frame(){}
    
    Frame(const std::tuple<std::vector<cv::KeyPoint>, cv::Mat> &features)
        : features(features)
    {
    }

    Eigen::Matrix4f GetPosition() const
    {
        return position;
    }

    const std::vector<cv::KeyPoint>& GetFeatures() const
    {
        return keypoints;
    }

    cv::Mat GetDescriptors() const
    {
        cv::Mat result(descriptors.size(), 32, descriptors[0].type());

        for (int i = 0; i < descriptors.size(); ++i)
        {
            result.row(i) = descriptors[i];
        }

        return result;
    }

    const std::vector<cv::Point3f>& GetPoints3d() const
    {
        return points;
    }

    bool isValid() const
    {
        return points.size() > 0;
    }

    void AddPoint(const int pointId, const cv::Point3f& point, const cv::KeyPoint& kp, const cv::Mat &descriptor)
    {
        points.push_back(point);
        keypoints.push_back(kp);
        descriptors.push_back(descriptor);
    }


protected:
    Eigen::Matrix4f position;
    std::tuple<std::vector<cv::KeyPoint>, cv::Mat> features;
    std::vector<cv::Point3f> points;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> descriptors;
};

} // namespace odometry
