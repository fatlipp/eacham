#include "sfm/reconstruction/BundleAdjuster.h"

#include "base/tools/Tools3d.h"
#include "sfm/reconstruction/Triangulator.h"

#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/PCGSolver.h>

#include <gtsam/slam/GeneralSFMFactor.h>

#include <concepts>
#include <tuple>
#include <unordered_set>

namespace eacham
{

gtsam::noiseModel::Diagonal::shared_ptr CreateNoise6_2_1(const float posNoise, const float rot)
{
    const float rotNoise = rot * 3.141592f / 180.0f;
    return gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << gtsam::Vector3::Constant(rotNoise), gtsam::Vector3::Constant(posNoise)).finished());  
}

void AddFrameToGraph()
{

}

void RefineBA(const int currentFrameId, 
    std::shared_ptr<graph_t> graph, 
    std::shared_ptr<Map> map, cv::Mat& K,
    const OptimizerConfig& config)
{
    // std::cout << "RefineBA()\n";

    gtsam::Cal3_S2 cameraMat(K.at<double>(0, 0), K.at<double>(1, 1), 
                                0.0, 
                                K.at<double>(0, 2) , K.at<double>(1, 2));

    gtsam::NonlinearFactorGraph graphGtsam;
    gtsam::Values initialMeasurements;

    std::vector<unsigned> frameIds;
    std::set<unsigned> mapIds;

    auto frameAdder = [&graphGtsam, &initialMeasurements, 
        &graph, &map, &mapIds](auto currentNode) {

        auto frameNoise = CreateNoise6_2_1(0.35, 45.0);
        const auto frameNoiseHuber = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(2.5f), 
            frameNoise);

        const Eigen::Matrix4d position = currentNode->GetTransform().inverse();

        initialMeasurements.insert(gtsam::Symbol('x', currentNode->GetId()), gtsam::Pose3(position));

        if (graph->IsFixed(currentNode->GetId()))
        {
            auto frameNoiseStatic = CreateNoise6_2_1(0.0001, 0.0001);
            graphGtsam.addPrior(gtsam::Symbol('x', currentNode->GetId()), gtsam::Pose3(position), frameNoiseStatic);
        }
        else
        {
            graphGtsam.addPrior(gtsam::Symbol('x', currentNode->GetId()), gtsam::Pose3(position), frameNoiseHuber);
        }

        unsigned countt = 0;
        unsigned countUnique = 0;

        for (const auto& [id2d, id3d] : currentNode->GetPoints3d())
        {
            if (!map->GetStatus(id3d) || map->GetObservers(id3d).size() < 2)
            {
                continue;
            }

            const auto noisePoint2dN = gtsam::noiseModel::Isotropic::Sigma(2, 1.5f);
            const auto noisePoint2d = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(3.0f), noisePoint2dN);

            const auto p2d = currentNode->GetKeyPoint(id2d);

            graphGtsam.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(gtsam::Point2{p2d.x, p2d.y}, noisePoint2d, 
                gtsam::Symbol('x', currentNode->GetId()), 
                gtsam::Symbol('l', id3d), 
                gtsam::Symbol('K', 0));

            if (mapIds.count(id3d) == 0)
            {
                mapIds.insert(id3d);

                const auto p3d = map->Get(id3d);
                const auto mapPointGTSAM = gtsam::Point3(p3d);

                initialMeasurements.insert(gtsam::Symbol('l', id3d), mapPointGTSAM);

                const auto obs = map->GetObservers(id3d).size();
                const auto noise3dPoint1N = gtsam::noiseModel::Isotropic::Sigma(3, 1.0f / obs);
                const auto noise3dPoint1 = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(3.0f / obs), 
                    noise3dPoint1N);
                graphGtsam.addPrior(gtsam::Symbol('l', id3d), mapPointGTSAM, noise3dPoint1);

                ++countUnique;
            }

            ++countt;
        }
    };

    if (currentFrameId > -1)
    {
        auto startNode = graph->Get(currentFrameId);
        frameIds.push_back(currentFrameId);
        frameAdder(startNode);

        for (const auto [id, factor] : startNode->GetFactors())
        {
            auto currentNode = graph->Get(id);

            if (currentNode == nullptr)
            {
                throw std::runtime_error("Node is null");
            }

            if (currentNode->IsValid())
            {
                frameIds.push_back(id);

                frameAdder(currentNode);
            }
        }
    }
    else
    {
        for (const auto [id, currentNode] : graph->GetNodes())
        {
            if (currentNode == nullptr)
            {
                throw std::runtime_error("Node is null");
            }

            if (currentNode->IsValid())
            {
                frameIds.push_back(id);

                frameAdder(currentNode);
            }
        }
    }

    std::cout << "BA() frames: " << frameIds.size() << ", map size: " << mapIds.size() << std::endl;

    if (mapIds.size() < 50)
    {
        return;
    }

    initialMeasurements.insert(gtsam::Symbol('K', 0), cameraMat);

    gtsam::noiseModel::Diagonal::shared_ptr cameraMatNoise = 
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 
            25, 25, 0.00001, 0.0001, 0.0001).finished());
        
    graphGtsam.emplace_shared<gtsam::PriorFactor<gtsam::Cal3_S2>>(gtsam::Symbol('K', 0), 
        cameraMat, cameraMatNoise);

    std::unique_ptr<gtsam::NonlinearOptimizer> optimizer;

    if (config.method == "LM")
    {
        gtsam::LevenbergMarquardtParams params;
        gtsam::LevenbergMarquardtParams::SetCeresDefaults(&params);
        // params.setVerbosityLM("SUMMARY");
        // params.orderingType = gtsam::Ordering::METIS;
        params.absoluteErrorTol = config.maxTolerance;
        params.relativeErrorTol = config.maxTolerance;
        params.maxIterations = config.maxIter;

        if (config.usePreconditioner)
        {
            params.linearSolverType = gtsam::NonlinearOptimizerParams::Iterative;
            gtsam::PCGSolverParameters::shared_ptr pcg = boost::make_shared<gtsam::PCGSolverParameters>();
            pcg->preconditioner_ = boost::make_shared<gtsam::BlockJacobiPreconditionerParameters>();
            pcg->setEpsilon_abs(1e-10);
            pcg->setEpsilon_rel(1e-10);
            params.iterativeParams = pcg;
        }

        optimizer = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graphGtsam, initialMeasurements, params);
    }
    else if (config.method == "DogLeg")
    {
        gtsam::DoglegParams params;
        // gtsam::DoglegParams::SetCeresDefaults(&params);
        // params.setVerbosityLM("SUMMARY");
        params.absoluteErrorTol = config.maxTolerance;
        params.relativeErrorTol = config.maxTolerance;
        params.maxIterations = config.maxIter;
        params.setDeltaInitial(config.delta);
        optimizer = std::make_unique<gtsam::DoglegOptimizer>(graphGtsam, initialMeasurements, params);
    }

    const gtsam::Values optimizationResult = optimizer->optimize();

    std::cout << "Optimization Initial error = " << graphGtsam.error(initialMeasurements) << std::endl;
    std::cout << "Optimization Final error = " << graphGtsam.error(optimizationResult) << std::endl << std::endl;

    const gtsam::Cal3_S2 newCamMat =
            optimizationResult.at<gtsam::Cal3_S2>(gtsam::Symbol('K', 0));

    K.at<double>(0, 0) = newCamMat.fx();
    K.at<double>(1, 1) = newCamMat.fy();
    K.at<double>(0, 2) = newCamMat.px();
    K.at<double>(1, 2) = newCamMat.py();

    for (const auto& id : mapIds)
    {
        const gtsam::Point3 mapPoint =
             optimizationResult.at<gtsam::Point3>(gtsam::Symbol('l', id));
        
        map->UpdatePoint(id, { mapPoint.x(), mapPoint.y(), mapPoint.z() });
        map->UpdateStatus(id, true);
    }

    for (int i = 0; i < frameIds.size(); ++i)
    {
        auto currentNode = graph->Get(frameIds[i]);

        const gtsam::Pose3 targetFrame = optimizationResult.at<gtsam::Pose3>(gtsam::Symbol('x', currentNode->GetId()));
        Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
        result.block<3, 3>(0, 0) = targetFrame.rotation().matrix();
        result.block<3, 1>(0, 3) = targetFrame.translation();
        result = result.inverse().eval();

        currentNode->SetTransform(result);
    }
}

}