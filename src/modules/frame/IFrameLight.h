#pragma once

#include "types/DataTypes.h"
#include "frame/FramePointData.h"

#include <opencv2/core.hpp>
#include <Eigen/Core>

namespace eacham
{

class IFrameLight
{

public:
    IFrameLight()
        : id(0)
        {
        }

    IFrameLight(const unsigned id, const std::vector<FramePointData>& pointsData)
        : id(id)
        , pointsData(pointsData)
        {}

public:
    const std::vector<FramePointData>& GetPointsData() const
    {
        return pointsData;
    }

    const FramePointData& GetPointData(const int id) const
    {
        return pointsData[id];
    }

    unsigned GetId() const
    {
        return id;
    }

    bool isValid()
    {
        return pointsData.size() > 0;
    }

private:
    unsigned id;
    std::vector<FramePointData> pointsData;
};

} // namespace eacham
