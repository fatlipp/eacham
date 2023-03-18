#include "MotionEstimatorOpt.h"
#include "tools/Tools3d.h"
#include "odometry/features/FeatureExtractor.h"

#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Geometry>

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

namespace eacham
{

MotionEstimatorOpt::MotionEstimatorOpt()
{
}

MotionEstimatorOpt::MotionEstimatorOpt(const cv::Mat &cameraMatInp, const cv::Mat &distCoeffs)
{
    if (cameraMatInp.rows == 1)
    {
        this->K = boost::make_shared<gtsam::Cal3_S2>(cameraMatInp.at<float>(0, 0), cameraMatInp.at<float>(0, 1), 
                                    0.0, 
                                    cameraMatInp.at<float>(0, 2) , cameraMatInp.at<float>(0, 3));
    }
    else
    {
        std::cerr << "NEED CHECK VALUES!!!!!!!!!\n";
    }
}

gtsam::noiseModel::Diagonal::shared_ptr CreateNoise6_2(const float posNoise, const float rot)
{
    const float rotNoise = rot * 3.141592f / 180.0f;
    return gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << gtsam::Vector3::Constant(rotNoise), gtsam::Vector3::Constant(posNoise)).finished());  
}

void addFramePointsToTheGraph(const std::vector<PointData>& points, const unsigned id, gtsam::NonlinearFactorGraph &graph, const gtsam::Cal3_S2::shared_ptr &K)
{
    const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.1);

    unsigned landmarkId = 0;
    for (auto &point : points)
    {
        const gtsam::Point2 measurement2 = {point.keypoint.pt.x, point.keypoint.pt.y};

        graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
            measurement2, measurementNoise, gtsam::Symbol('x', id), gtsam::Symbol('l', landmarkId), K);

        ++landmarkId;
    }
}

std::tuple<Eigen::Matrix4f, unsigned> MotionEstimatorOpt::Estimate(const Frame& frame1, Frame& frame2)
{
    const cv::Mat descriptor1 = frame1.GetDescriptors().clone();
    const cv::Mat descriptor2 = frame2.GetDescriptors().clone();

    const static matcher_t mather = FeatureExtractor::GetMatcher();

    std::vector<std::vector<cv::DMatch>> matches;
    mather->knnMatch(descriptor1, descriptor2, matches, 2);

    if (matches.size() < 1)
    {
        return {Eigen::Matrix4f::Identity(), 0};
    }

    std::vector<PointData> pts3d1; 
    std::vector<PointData> pts3d2; 

    for (const auto& m : matches)
    {
        if (m[0].distance < 0.7f * m[1].distance)
        {
            pts3d1.push_back(frame1.GetPointData(m[0].queryIdx));
            pts3d2.push_back(frame2.GetPointData(m[0].trainIdx));
        }
    }

    const unsigned inliers = pts3d1.size();

    std::cout << "MotionEstimationOpt() Good matches: " << inliers << std::endl;

    if (inliers < 10)
    {
        std::cout << "Motion Estimation error: Not enough matches count\n";

        return {Eigen::Matrix4f::Identity(), 0};
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialMeasurements;

    const int INITIAL_ID = 0;
    int frameId = INITIAL_ID;

    const Eigen::Matrix4d position = Eigen::Matrix4d::Identity();
    graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(gtsam::Symbol('x', frameId), gtsam::Pose3(position));
    initialMeasurements.insert(gtsam::Symbol('x', frameId), gtsam::Pose3(position));
    
    ++frameId;
    const auto noise2 = CreateNoise6_2(3.0, 15.0);
    graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise2);
    initialMeasurements.insert(gtsam::Symbol('x', frameId), gtsam::Pose3(position));

    addFramePointsToTheGraph(pts3d1, 0, graph, this->K);
    addFramePointsToTheGraph(pts3d2, 1, graph, this->K);
    
    // map
    unsigned landmarkId = 0;
    for (auto &point1 : pts3d1)
    {
        const auto point = point1.point3d;

        const auto mapPointGTSAM = gtsam::Point3( point.position.x, point.position.y, point.position.z );
        initialMeasurements.insert(gtsam::Symbol('l', landmarkId), mapPointGTSAM);

        // add uncertatinty to the map points
        const auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.15);
        graph.addPrior(gtsam::Symbol('l', landmarkId), mapPointGTSAM, priorNoise);

        ++landmarkId;
    }


    std::unique_ptr<gtsam::NonlinearOptimizer> optimizer;

    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = 1000;
    params.absoluteErrorTol = 0.000001;
    // params.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    // params.linearSolverType = gtsam::NonlinearOptimizerParams::CHOLMOD;
    optimizer = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph, initialMeasurements, params);

    gtsam::Values optimizationResult = optimizer->optimize();

    std::cout << "frames: " << frameId << ", mapPoints: " << landmarkId << std::endl;
    std::cout << "++initial error = " << graph.error(initialMeasurements) << std::endl;
    std::cout << "++final error = " << graph.error(optimizationResult) << std::endl << std::endl;

    gtsam::Pose3 frame11 = optimizationResult.at<gtsam::Pose3>(gtsam::Symbol('x', 1));
    const auto rr = frame11.rotation();
    std::cout << "frame " << 1 << " = \n" << rr.matrix() << "\n" << frame11.translation() << std::endl;
    
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = rr.matrix();
    result.block<3, 1>(0, 3) = frame11.translation();

    return { result.cast<float>(), inliers };
}

}