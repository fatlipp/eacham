#pragma once

#include "IMotionEstimator.h"

#include "frame/IFrame.h"
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
    std::tuple<std::vector<int>, std::vector<int>> FindMatches(const IFrame& frame1, const IFrame& frame2);

protected:
    cv::Mat cameraMat;
    cv::Mat distCoeffs;

private:
    matcher_t mather;

};

}
