#include "MotionEstimatorOpt.h"
#include "tools/Tools3d.h"

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

MotionEstimatorOpt::MotionEstimatorOpt(const FeatureExtractorType &featureExtractor, const cv::Mat &cameraMatInp, const cv::Mat &distCoeffsInp)
    : MotionEstimatorBase(featureExtractor)
{
    if (cameraMatInp.rows == 1)
    {
        this->K = boost::make_shared<gtsam::Cal3_S2>(cameraMatInp.at<float>(0, 0), cameraMatInp.at<float>(0, 1), 
                                    0.0, 
                                    cameraMatInp.at<float>(0, 2) , cameraMatInp.at<float>(0, 3));
        cameraMat = cv::Mat::eye(3, 3, CV_32FC1);
        cameraMat.at<float>(0, 0) = cameraMatInp.at<float>(0, 0);
        cameraMat.at<float>(1, 1) = cameraMatInp.at<float>(0, 1);
        cameraMat.at<float>(0, 2) = cameraMatInp.at<float>(0, 2);
        cameraMat.at<float>(1, 2) = cameraMatInp.at<float>(0, 3);
        cameraMat.at<float>(2, 2) = 1.0f;

        this->cameraMatOneDim = cameraMatInp.clone();
    }
    else
    {
        std::cerr << "NEED CHECK VALUES!!!!!!!!!\n";
    }

    this->distCoeffs = distCoeffsInp;
}

gtsam::noiseModel::Diagonal::shared_ptr CreateNoise6_2(const float posNoise, const float rot)
{
    const float rotNoise = rot * 3.141592f / 180.0f;
    return gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << gtsam::Vector3::Constant(rotNoise), gtsam::Vector3::Constant(posNoise)).finished());  
}

void addFramePointsToTheGraph(const Frame &frame, const std::vector<int>& pointsIds, const unsigned id, 
    gtsam::NonlinearFactorGraph &graph, const gtsam::Cal3_S2::shared_ptr &K)
{
    const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.1);

    unsigned landmarkId = 0;
    for (auto &pointId : pointsIds)
    {
        const auto point = frame.GetPointData(pointId).keypoint.pt;
        const gtsam::Point2 measurement2 = {point.x, point.y};

        graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
            measurement2, measurementNoise, gtsam::Symbol('x', id), gtsam::Symbol('l', landmarkId), K);

        ++landmarkId;
    }
}

std::tuple<Eigen::Matrix4f, unsigned> MotionEstimatorOpt::Estimate(const Frame& frame1, Frame& frame2)
{
    const auto [pts1, pts2] = FindMatches(frame1, frame2); 

    const unsigned matches = pts1.size();

    for (size_t i = 0; i < matches; ++i)
    {
        frame2.GetPointData(pts2[i]).associatedMapPointId = frame1.GetPointData(pts1[i]).associatedMapPointId;
    }

    std::cout << "MotionEstimationOpt() Good matches: " << matches << std::endl;

    if (matches < 10)
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
    const auto noise2 = CreateNoise6_2(5.0, 25.0);
    graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise2);
    initialMeasurements.insert(gtsam::Symbol('x', frameId), gtsam::Pose3(position));

    addFramePointsToTheGraph(frame1, pts1, 0, graph, this->K);
    addFramePointsToTheGraph(frame2, pts2, 1, graph, this->K);
    
    // map
    unsigned landmarkId = 0;
    for (auto &point1 : pts1)
    {
        const auto point = frame1.GetPointData(point1).position3d;

        const auto mapPointGTSAM = gtsam::Point3( point.x, point.y, point.z );
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
    optimizer = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph, initialMeasurements, params);

    const gtsam::Values optimizationResult = optimizer->optimize();

    std::cout << "frames: " << frameId << ", mapPoints: " << landmarkId << std::endl;
    std::cout << "Optimization Initial error = " << graph.error(initialMeasurements) << std::endl;
    std::cout << "Optimization Final error = " << graph.error(optimizationResult) << std::endl << std::endl;

    const gtsam::Pose3 targetFrame = optimizationResult.at<gtsam::Pose3>(gtsam::Symbol('x', 1));
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = targetFrame.rotation().matrix();
    result.block<3, 1>(0, 3) = targetFrame.translation();

    {
        std::vector<cv::Point3f> pts3d1;
        std::vector<cv::Point2f> pts2d2; 

        for (size_t i = 0; i < matches; ++i)
        {
            pts3d1.push_back(frame1.GetPointData(pts1[i]).position3d);
            pts2d2.push_back(frame2.GetPointData(pts2[i]).keypoint.pt);
        }

        // reprojection error stat
        {
            cv::Mat Rmat = cv::Mat_<double>(3, 3);

            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    Rmat.at<double>(i, j) = result(i, j); 
                }
            }

            cv::Mat tvec = cv::Mat_<double>(3, 1);
            tvec.at<double>(0, 0) = targetFrame.translation().x();
            tvec.at<double>(1, 0) = targetFrame.translation().y();
            tvec.at<double>(2, 0) = targetFrame.translation().z();
            // CalcReprojectionError(frame2.GetImage(), pts3d1, pts2d2, Rmat, tvec);
            std::vector<int> reprojectedInliers;
            const auto [errMean1, errVar1] = CalcReprojectionError(frame2.GetImage(), pts3d1, pts2d2, Rmat, tvec, 8.0f, reprojectedInliers);
            std::cout << "inliers (reprojected): " << reprojectedInliers.size() << " (" << (reprojectedInliers.size() / static_cast<float>(matches)) << ")" << std::endl;

        }
    }

    return { result.cast<float>(), matches };
}

}