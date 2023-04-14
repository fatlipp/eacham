#pragma once

#include "types/DataTypes.h"

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

namespace eacham
{

struct PointData
{
    unsigned id = 0;
    unsigned associatedMapPointId = 0;
    float uncertatinty;

    cv::KeyPoint keypoint;
    cv::Point3f position3d;
    cv::Mat descriptor;

    PointData(const cv::KeyPoint &keypoint, const cv::Point3f &position3d, const cv::Mat &descriptor)
        : id(0)
        , uncertatinty(1000.0f)
        , associatedMapPointId(0)
        , keypoint(keypoint)
        , position3d(position3d)
        , descriptor(descriptor.clone())
        {}
};

class Frame
{
public:
    Frame()
        : timestamp(-1.0)
    {}
    
    Frame(const double timestamp)
        : timestamp(timestamp)
    {
    }

    cv::KeyPoint GetFeature(const int id) const
    {
        return pointsData[id].keypoint;
    }

    cv::Point3f GetPoint3d(const int id) const
    {
        return pointsData[id].position3d;
    }

    const std::vector<PointData>& GetPointsData() const
    {
        return pointsData;
    }

    cv::Mat GetDescriptors() const
    {
        cv::Mat result = cv::Mat(pointsData.size(), pointsData[0].descriptor.cols, pointsData[0].descriptor.type());

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

    void AddPoint(const int pointId, const cv::Point3f& position3d, const cv::KeyPoint& keypoint, const cv::Mat &descriptor)
    {
        pointsData.push_back({keypoint, position3d, descriptor.clone()});
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

    PointData& GetPointData(const int id)
    {
        return pointsData[id];
    }

protected:
    double timestamp;

    Eigen::Matrix4f position;
    Eigen::Matrix4f odometry;
    std::vector<PointData> pointsData;

};

} // namespace eacham
