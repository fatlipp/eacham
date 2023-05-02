#pragma once

#include "types/DataTypes.h"

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

namespace eacham
{

struct KeyPointDataVisual
{
    unsigned associatedMapPointId = 0;

    cv::Point2f keypoint;
    cv::Point3f position3d;
    cv::Mat descriptor;

    KeyPointDataVisual(const cv::Point2f &keypoint, const cv::Point3f &position3d, const cv::Mat &descriptor)
        : associatedMapPointId(0)
        , keypoint(keypoint)
        , position3d(position3d)
        , descriptor(descriptor.clone())
        {}
};

class IFrame
{

public:
    IFrame()
        : id(0)
        {}

public:
    bool isValid() const
    {
        return pointsData.size() > 0;
    }

public:
    const std::vector<KeyPointDataVisual>& GetPointsData() const
    {
        return pointsData;
    }
    
    std::vector<KeyPointDataVisual>& GetPointsData()
    {
        return pointsData;
    }

    Eigen::Matrix4f GetOdometry() const
    {
        return this->odometry;
    }

    Eigen::Matrix4f GetPosition() const
    {
        return this->position;
    }

    KeyPointDataVisual GetPointData(const int id) const
    {
        return pointsData[id];
    }

    KeyPointDataVisual& GetPointData(const int id)
    {
        return pointsData[id];
    }

    unsigned GetId() const
    {
        return id;
    }

public:
    void AddPoint(const KeyPointDataVisual& data)
    {
        this->pointsData.push_back(data);
    }

    void SetOdometry(const Eigen::Matrix4f &odom) 
    {
        this->odometry = odom;
    }

    void SetPosition(const Eigen::Matrix4f &position) 
    {
        this->position = position;
    }

    void SetId(const unsigned id) 
    {
        this->id = id;
    }

protected:
    Eigen::Matrix4f position;
    Eigen::Matrix4f odometry;
    std::vector<KeyPointDataVisual> pointsData;

    unsigned id;

};

} // namespace eacham
