
#pragma once

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

namespace eacham
{

struct KeyPointDataVisual
{
    unsigned mapPointId;
    cv::Point2f keypoint;
};

struct MapFrame
{
    unsigned id = 0;
    unsigned parentId = 0;
    Eigen::Matrix4f odometry;
    Eigen::Matrix4f position;
    std::unordered_map<unsigned, KeyPointDataVisual> pointsData;

    void AddPoint(const unsigned pointId, const KeyPointDataVisual& point)
    {
        pointsData.insert({pointId, point});
    }

    bool isValid() const
    {
        return id > 0;
    }

};

}