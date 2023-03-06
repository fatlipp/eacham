#pragma once

#include "types/DataTypes.h"

#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Core>

namespace odometry
{

struct PointData
{
    cv::KeyPoint keypoint;
    cv::Point3f point3d;
    cv::Mat descriptor;

    PointData(const cv::KeyPoint &keypoint, const cv::Point3f &point3d, const cv::Mat &descriptor)
        : keypoint(keypoint)
        , point3d(point3d)
        , descriptor(descriptor.clone())
        {}
};

class Frame
{
public:
    Frame()
        : timestamp(-1.0)
    {}
    
    Frame(const double timestamp, const cv::Mat &imageInp, const std::tuple<std::vector<cv::KeyPoint>, cv::Mat> &features)
        : timestamp(timestamp)
    {
        imageInp.copyTo(image);
    }

    cv::Mat GetImage() const
    {
        return image.clone();
    }

    cv::KeyPoint GetFeature(const int id) const
    {
        return pointsData[id].keypoint;
    }

    cv::Point3f GetPoint3d(const int id) const
    {
        return pointsData[id].point3d;
    }

    cv::Mat GetDescriptors() const
    {
        cv::Mat result = cv::Mat(pointsData.size(), 32, pointsData[0].descriptor.type());

        for (int i = 0; i < pointsData.size(); ++i)
        {
            pointsData[i].descriptor.copyTo(result.row(i));
        }

        return result;
    }

    bool isValid() const
    {
        return pointsData.size() > 0;
    }

    size_t GetSize() const
    {
        return pointsData.size();
    }

    void AddPoint(const int pointId, const cv::Point3f& point3d, const cv::KeyPoint& keypoint, const cv::Mat &descriptor)
    {
        pointsData.push_back({keypoint, point3d, descriptor.clone()});
    }

    double timestamp;

protected:
    Eigen::Matrix4f position;
    cv::Mat image;
    std::vector<PointData> pointsData;
};

} // namespace odometry
