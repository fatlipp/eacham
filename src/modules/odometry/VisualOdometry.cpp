
#include "VisualOdometry.h"

namespace odometry
{

Eigen::Matrix4f VisualOdometry::GetOdometry(const data_t &data)
{
    const auto [features1, descriptor1] = extractor.GetFeatures(std::get<0>(data));
    const auto [features2, descriptor2] = extractor.GetFeatures(std::get<1>(data));

    return Eigen::Matrix4f::Identity();
}

} // namespace odometry
