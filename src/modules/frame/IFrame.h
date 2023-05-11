#pragma once

#include "types/DataTypes.h"
#include "frame/FramePointData.h"

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

namespace eacham
{

class IFrame
{

public:
    IFrame()
        : id(0)
        , position{Eigen::Matrix4f::Identity()}
        {}

public:
    bool isValid() const
    {
        return pointsData.size() > 0;
    }

public:
    std::vector<FramePointData> GetPointsDataCopy() const
    {
        return pointsData;
    }

    const std::vector<FramePointData>& GetPointsData() const
    {
        return pointsData;
    }
    
    std::vector<FramePointData>& GetPointsData()
    {
        return pointsData;
    }

    Eigen::Matrix4f GetPosition() const
    {
        return this->position;
    }

    const FramePointData& GetPointData(const int id) const
    {
        return pointsData[id];
    }

    FramePointData& GetPointData(const int id)
    {
        return pointsData[id];
    }

    unsigned GetId() const
    {
        return id;
    }

public:
    void AddPoint(const FramePointData& data)
    {
        this->pointsData.push_back(data);
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
    std::vector<FramePointData> pointsData;

    unsigned id;

};

} // namespace eacham
