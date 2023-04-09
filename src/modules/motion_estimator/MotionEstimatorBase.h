#pragma once

#include "IMotionEstimator.h"

#include "frame/Frame.h"
#include "types/DataTypes.h"

namespace eacham
{

class MotionEstimatorBase : public IMotionEstimator
{
public:
    MotionEstimatorBase()
    {
    }

    void SetMatcher(const matcher_t& matcherInp)
    {
        this->mather = matcherInp;
    }

public:
    std::tuple<std::vector<int>, std::vector<int>> FindMatches(const Frame& frame1, const Frame& frame2);

protected:
    cv::Mat cameraMat;
    cv::Mat distCoeffs;

private:
    matcher_t mather;

};

}
