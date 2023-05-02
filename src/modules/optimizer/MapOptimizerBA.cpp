#include "optimizer/MapOptimizerBA.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
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

#include "tools/Tools3d.h"

#include <map>
#include <algorithm>
#include <ranges>
#include <boost/range/irange.hpp>

namespace eacham
{
    gtsam::noiseModel::Diagonal::shared_ptr CreateNoise6(const float posNoise, const float rot)
    {
        const float rotNoise = rot * 3.141592f / 180.0f;
        return gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(6) << gtsam::Vector3::Constant(rotNoise), gtsam::Vector3::Constant(posNoise)).finished());  
    }
}

namespace eacham
{
    MapOptimizerBA::MapOptimizerBA(const ConfigMapOptimizer& config, const cv::Mat &cameraMatInp)
        : IMapOptimizer(config)
    {
        if (cameraMatInp.rows == 1)
        {
            this->cameraMat = boost::make_shared<gtsam::Cal3_S2>(
                    cameraMatInp.at<float>(0, 0), cameraMatInp.at<float>(0, 1), 
                    0.0, 
                    cameraMatInp.at<float>(0, 2) , cameraMatInp.at<float>(0, 3));
        }
        else
        {
            std::cerr << "IMapOptimizer() Wrong camera parameters\n";
        }
    }

    std::unique_ptr<gtsam::NonlinearOptimizer> MakeOptimizer(const ConfigMapOptimizer& config,
        const gtsam::NonlinearFactorGraph& graph, 
        const gtsam::Values& initialMeasurements)
    {
        const unsigned type = config.GetType();

        if (type == 0)
        {
            gtsam::LevenbergMarquardtParams params;
            params.lambdaFactor = 3;
            params.maxIterations = config.GetMaxIterations();
            params.absoluteErrorTol = 0.01;

            return std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph, initialMeasurements, params);
        }
        
        if (type == 1)
        {
			gtsam::GaussNewtonParams params;
			params.relativeErrorTol = 0.01f;
			params.maxIterations = config.GetMaxIterations();

			return std::make_unique<gtsam::GaussNewtonOptimizer>(graph, initialMeasurements, params);
        }
        
        if (type == 2)
        {
			gtsam::DoglegParams params;
			params.relativeErrorTol = 0.01f;
			params.maxIterations = config.GetMaxIterations();

			return std::make_unique<gtsam::DoglegOptimizer>(graph, initialMeasurements, params);
        }

        return {};
    }
    
    bool MapOptimizerBA::Optimize()
    {
        std::cout << "MapOptimizerBA() map.size = " << this->map->GetSize() << std::endl;

        if (this->map->GetSize() < 2)
        {
            return false;
        }

        if (this->onStart != nullptr)
        {
            this->onStart();
        }

        // std::lock_guard<IMap> lock(*(this->map));
        
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialMeasurements;

        std::vector<int> frameIds;
        std::set<int> mapPointIds;

        const auto frames = this->map->GetFrames();
        std::for_each(frames.begin(), frames.end(), [&](const IFrame& frame)
            {
                const auto frameId = frame.GetId();
                const Eigen::Matrix4d position = frame.GetPosition().cast<double>();
                initialMeasurements.insert(gtsam::Symbol('x', frameId), gtsam::Pose3(position));

                frameIds.push_back(frameId);
                
                // the first frame is static
                if (frameId > 1)
                {
                    // const auto noiseOdom = CreateNoise6(config.GetOdomNoiseRot(), config.GetOdomNoisePos());
                    // const Eigen::Matrix4d odom = frame.GetOdometry().cast<double>();
                    // graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('x', frameId), gtsam::Symbol('x', frameId - 1), 
                    //     gtsam::Pose3(odom), noiseOdom);

                    const auto noise = CreateNoise6(config.GetKeyframeNoiseRot(), config.GetKeyframeNoisePos());
                    graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise);
                }
                else
                {
                    const auto noise = CreateNoise6(0.000001, 0.0000001); 
                    graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise);
                }

                for (unsigned count = 0; const auto &point : frame.GetPointsData())
                {
                    if (count > config.GetPointsLimit())
                    {
                        break;
                    }

                    if (point.associatedMapPointId == 0 || map->GetPoint(point.associatedMapPointId).observers < 2)
                        continue;

                    ++count;

                    mapPointIds.insert(point.associatedMapPointId);

                    // const auto mapPoint = map.GetPoint(point.point3d.mapPointId).position;
                    // const auto mapPointGTSAM = gtsam::Point3( mapPoint.x, mapPoint.y, mapPoint.z );
                    // gtsam::PinholeCamera<gtsam::Cal3_S2> camera(gtsam::Pose3(position), *K);
                    // const gtsam::Point2 measurement0 = camera.project2(mapPointGTSAM);
                    // const gtsam::Point2 measurement1 = camera.projectSafe(mapPointGTSAM).first;
                    const gtsam::Point2 measurement2 = {point.keypoint.x, point.keypoint.y};

                    // std::cout << "measurement = 1: [" << measurement0.x() << ", " << measurement0.y() << 
                    //     "], 2: [" << measurement1.x() << ", " << measurement1.y() << 
                    //     "], 3: [" << measurement2.x() << ", " << measurement2.y() << 
                    //     "], map: [" << mapPointGTSAM.x() << ", " << mapPointGTSAM.y() << ", " << mapPointGTSAM.z() << "] " << std::endl;

                    // const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, point.uncertatinty);
                    const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, config.GetMeasurementNoiseUv());
                    const auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(config.GetHuberUv()), 
                        measurementNoise);
                    
                    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
                        measurement2, huber, gtsam::Symbol('x', frameId), gtsam::Symbol('l', point.associatedMapPointId), this->cameraMat);

                }
            });
        
        // map
        std::for_each(mapPointIds.begin(), mapPointIds.end(), [&](const auto& id)
            {
                const auto point = this->map->GetPoint(id);
                const auto mapPointGTSAM = gtsam::Point3( point.position.x, point.position.y, point.position.z );
                initialMeasurements.insert(gtsam::Symbol('l', id), mapPointGTSAM);

                // add uncertatinty to the map points
                const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(3, config.GetMeasurementNoise3d());
                const auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(config.GetHuber3d()), 
                    measurementNoise);

                graph.addPrior(gtsam::Symbol('l', id), mapPointGTSAM, huber);
            });

        auto optimizer = MakeOptimizer(config, graph, initialMeasurements);
        const gtsam::Values optimizationResult = optimizer->optimize();

        std::cout << "frames: " << frameIds.size() << ", mapPoints: " << mapPointIds.size() << std::endl;
        std::cout << "++initial error = " << graph.error(initialMeasurements) << std::endl;
        std::cout << "++final error = " << graph.error(optimizationResult) << std::endl << std::endl;

        std::for_each(mapPointIds.begin(), mapPointIds.end(), [&](const auto& id)
            {
                const gtsam::Point3 mapPoint = optimizationResult.at<gtsam::Point3>(gtsam::Symbol('l', id));
                cv::Point3f result { 
                        static_cast<float>(mapPoint.x()), 
                        static_cast<float>(mapPoint.y()), 
                        static_cast<float>(mapPoint.z())
                    };

                // std::cout << "ID: " << id << ", BEFORE: " << map->GetPoint(id).position;
                this->map->GetPoint(id).position = result;
                // std::cout << ", AFTER: " << map->GetPoint(id).position << std::endl;
            });

        // const auto frameRanges = std::views::iota(0, frameId);
        std::for_each(frameIds.begin(), frameIds.end(), [&](const auto& id)
            {
                const gtsam::Pose3 targetFrame = optimizationResult.at<gtsam::Pose3>(gtsam::Symbol('x', id));
                Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
                result.block<3, 3>(0, 0) = targetFrame.rotation().matrix();
                result.block<3, 1>(0, 3) = targetFrame.translation();

                auto& frame = this->map->GetFrame(id);

                if (!frame.isValid())
                {
                    return;
                }

                // std::cout << "ID: " << i << ", BEFORE:\n" << map->GetFrame(id).GetPosition() << std::endl;
                frame.SetPosition(result.cast<float>());
                // std::cout << "AFTER:\n" << map->GetFrame(id).GetPosition() << std::endl;

                // for (auto &point : frame.GetPointsData())
                // {
                //     if (point.associatedMapPointId == 0)
                //         continue;

                //     point.position3d = this->map->GetPoint(point.associatedMapPointId).position;
                //     point.position3d = tools::transformPoint3d(point.position3d, frame.GetPosition().inverse());             
                // }
            });

        if (this->onComplete != nullptr)
        {
            this->onComplete();
        }

        return true;
    }
} // namespace eacham
