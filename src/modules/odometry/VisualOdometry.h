#pragma once

#include "IOdometry.h"
#include "types/DataTypes.h"
#include "features/FeatureExtractor.h"

namespace odometry
{
using data_t = stereodata_t;

class VisualOdometry : public IOdometry<data_t>
{
public:
    VisualOdometry()
    {
    }

    Eigen::Matrix4f GetOdometry(const data_t &data) override;

private:
    FeatureExtractor extractor;
};

} // namespace odometry
