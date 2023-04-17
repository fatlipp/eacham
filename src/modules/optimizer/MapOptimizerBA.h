#pragma once

#include "optimizer/IMapOptimizer.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace eacham
{

class MapOptimizerBA : public IMapOptimizer
{
public:
    MapOptimizerBA(const cv::Mat &cameraMatInp);

    bool Optimize() override;

private:
    boost::shared_ptr<gtsam::Cal3_S2> cameraMat;
};

}