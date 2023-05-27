#include "MotionEstimatorOpt.h"
#include "tools/Tools3d.h"

#include "motion_estimator/MotionEstimatorTools.h"

#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <concepts>
#include <tuple>
#include <unordered_set>

namespace eacham
{

MotionEstimatorOpt::MotionEstimatorOpt(const cv::Mat &cameraMatInp, const cv::Mat &distCoeffsInp)
    : MotionEstimatorBase(cameraMatInp, distCoeffsInp)
{
    if (cameraMatInp.rows == 1)
    {
        this->cameraMatGtsam = boost::make_shared<gtsam::Cal3_S2>(cameraMatInp.at<float>(0, 0), cameraMatInp.at<float>(0, 1), 
                                    0.0, 
                                    cameraMatInp.at<float>(0, 2) , cameraMatInp.at<float>(0, 3));
    }
}

gtsam::noiseModel::Diagonal::shared_ptr CreateNoise6_2(const float posNoise, const float rot)
{
    const float rotNoise = rot * 3.141592f / 180.0f;
    return gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << gtsam::Vector3::Constant(rotNoise), gtsam::Vector3::Constant(posNoise)).finished());  
}

template<typename T>
concept IsFrame = requires(const T& frame)
{
    { frame.GetPointData(0) } -> std::same_as<const FramePointData&>;
};

template<unsigned ID, IsFrame T>
void AddFramePointsToTheGraph(const T& frame, const std::vector<std::pair<unsigned, unsigned>>& matches, 
    gtsam::NonlinearFactorGraph &graph, const gtsam::Cal3_S2::shared_ptr &camera)
{
    // const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.1);
    const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 2.0f);
    const auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(5.0f), 
        measurementNoise);

    for (unsigned landmarkId = 0; const auto &math : matches)
    {
        const unsigned idd = ID;
        const auto pointId = std::get<ID>(math);
        const auto point = frame.GetPointData(pointId).keypoint;
        const gtsam::Point2 measurement2 = {point.x, point.y};

        graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
            measurement2, huber, gtsam::Symbol('x', idd), gtsam::Symbol('l', landmarkId), camera);

        ++landmarkId;
    }
}

EstimationResult MotionEstimatorOpt::Estimate(const IFrameLight& frame1, const IFrame& frame2,
    const std::vector<std::pair<unsigned, unsigned>>& matches)
{
    const unsigned matchesSize = matches.size();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialMeasurements;

    int frameId = 0;

    Eigen::Matrix4d position = Eigen::Matrix4d::Identity();
    graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(gtsam::Symbol('x', frameId), gtsam::Pose3(position));
    initialMeasurements.insert(gtsam::Symbol('x', frameId), gtsam::Pose3(position));

    ++frameId;
    const auto noise2 = CreateNoise6_2(0.5, 5.0);
    graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise2);
    initialMeasurements.insert(gtsam::Symbol('x', frameId), gtsam::Pose3(position));

    AddFramePointsToTheGraph<0>(frame1, matches, graph, this->cameraMatGtsam);
    AddFramePointsToTheGraph<1>(frame2, matches, graph, this->cameraMatGtsam);
    
    // map
    unsigned landmarkId = 0;
    for (const auto &[point1, point2] : matches)
    {
        const auto point = frame1.GetPointData(point1).position3d;

        const auto mapPointGTSAM = gtsam::Point3( point.x, point.y, point.z );
        initialMeasurements.insert(gtsam::Symbol('l', landmarkId), mapPointGTSAM);

        // add uncertatinty to the map points
        const auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
        graph.addPrior(gtsam::Symbol('l', landmarkId), mapPointGTSAM, priorNoise);

        ++landmarkId;
    }

    std::unique_ptr<gtsam::NonlinearOptimizer> optimizer;

    gtsam::LevenbergMarquardtParams params;
    gtsam::LevenbergMarquardtParams::SetCeresDefaults(&params);
    // params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
    params.setMaxIterations(4);
    optimizer = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph, initialMeasurements, params);

    const gtsam::Values optimizationResult = optimizer->optimize();

    std::cout << "frames: " << frameId << ", mapPoints: " << landmarkId << std::endl;
    std::cout << "Optimization Initial error = " << graph.error(initialMeasurements) << std::endl;
    std::cout << "Optimization Final error = " << graph.error(optimizationResult) << std::endl << std::endl;

    const gtsam::Pose3 targetFrame = optimizationResult.at<gtsam::Pose3>(gtsam::Symbol('x', 1));
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = targetFrame.rotation().matrix();
    result.block<3, 1>(0, 3) = targetFrame.translation();

    EstimationResult estimation;
    estimation.frameIdPrev = frame1.GetId();
    estimation.frameIdCurrent = frame2.GetId();
    estimation.odometry = result.cast<float>();

    // TODO: reprojection best candidates
    for (const auto& match : matches)
    {
        const auto prevFramePointId = frame1.GetPointData(match.first).id;
        const auto currentFramePointId = frame2.GetPointData(match.second).id;
        estimation.matches.insert({currentFramePointId, prevFramePointId});
    }

    return estimation;
}

}