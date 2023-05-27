#pragma once

#include <Eigen/Core>

namespace eacham
{

struct EstimationResult
{
    unsigned frameIdPrev;
    unsigned frameIdCurrent;
    Eigen::Matrix4f odometry;
    std::unordered_map<unsigned, unsigned> matches;

    bool isValid() const
    {
        return frameIdCurrent > 0;
    }
};

}