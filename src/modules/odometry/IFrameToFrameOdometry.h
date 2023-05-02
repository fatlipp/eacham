#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "odometry/IVisualOdometry.h"
#include "types/DataTypes.h"
#include "map/IMap.h"
#include "map/LocalMap.h"

namespace eacham
{

template<typename T>
class IFrameToFrameOdometry : public IVisualOdometry<T>
{
};

} // namespace eacham