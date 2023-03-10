#include "LocalFramesOptimizer.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace odometry
{
    LocalFramesOptimizer::LocalFramesOptimizer()
    {
    }

    bool LocalFramesOptimizer::Optimize(LocalMap &map)
    {
        if (map.size() < 3)
        {
            return false;
        }
        
        gtsam::NonlinearFactorGraph graph;
        gtsam::Pose3 first_pose;
        // graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(0, gtsam::Pose3());

        const auto noiseModel = gtsam::noiseModel::Isotropic::Sigma(6, 3);

        gtsam::Values framePoses;

        const int INITIAL_ID = 1;
        int id = INITIAL_ID;
        for (auto &frame : map.GetFrames())
        {
            const Eigen::Matrix4d position = frame.GetPosition().cast<double>();
            framePoses.insert(id, gtsam::Pose3(position));

            if (id > INITIAL_ID)
            {
                const Eigen::Matrix4d odom = frame.GetOdometry().cast<double>();
                graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(id - 1, id, gtsam::Pose3(odom), noiseModel);
                
            }

            // map
            for (auto &point : frame.GetPointsData())
            {
                auto mapPoint = map.GetPoint(point.point3d.mapPointId);


            }


            ++id;
        }

        gtsam::LevenbergMarquardtOptimizer optimizer(graph, framePoses);
        gtsam::Values result = optimizer.optimize();

        result.print("Optimization result:\n");

        return true;
    }
} // namespace odometry
