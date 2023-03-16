#pragma once

#include "types/DataTypes.h"

#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Core>

namespace odometry
{

struct FramePoint3d
{
    unsigned mapPointId = 0;
    cv::Point3f position;
    unsigned observers = 0;

    FramePoint3d(const unsigned mapPointId, const cv::Point3f &position)
        : mapPointId(mapPointId)
        , position(position)
        {}

    void SetMapPointId(const unsigned mapPointId)
    {
        this->mapPointId = mapPointId;
    }
};

struct PointData
{
    unsigned id;

    cv::KeyPoint keypoint;
    FramePoint3d point3d;
    cv::Mat descriptor;

    PointData(const cv::KeyPoint &keypoint, const FramePoint3d &point3d, const cv::Mat &descriptor)
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

    FramePoint3d GetPoint3d(const int id) const
    {
        return pointsData[id].point3d;
    }

    std::vector<PointData>& GetPointsData()
    {
        return pointsData;
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

    double GetTimestamp() const
    {
        return timestamp;
    }

    void AddPoint(const int pointId, const cv::Point3f& point3d, const cv::KeyPoint& keypoint, const cv::Mat &descriptor)
    {
        pointsData.push_back({keypoint, {0, point3d}, descriptor.clone()});
    }

    void SetOdometry(const Eigen::Matrix4f &odom) 
    {
        this->odometry = odom;
    }

    void SetPosition(const Eigen::Matrix4f &position) 
    {
        this->position = position;
    }

    Eigen::Matrix4f GetOdometry() const
    {
        return this->odometry;
    }

    Eigen::Matrix4f GetPosition() const
    {
        return this->position;
    }

    void SetPointsData(const std::vector<PointData> &data)
    {
        this->pointsData = data;
    }

    PointData GetPointData(const int id) const
    {
        return pointsData[id];
    }

protected:
    double timestamp;

    Eigen::Matrix4f position;
    Eigen::Matrix4f odometry;
    cv::Mat image;
    std::vector<PointData> pointsData;

};

} // namespace odometry
