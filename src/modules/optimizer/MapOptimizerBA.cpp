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
    MapOptimizerBA::MapOptimizerBA(const cv::Mat &cameraMatInp)
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
    
    bool MapOptimizerBA::Optimize()
    {
        std::cout << "MapOptimizerBA() map.size = " << this->map->GetSize() << std::endl;

        if (this->map->GetSize() < 2)
        {
            return false;
        }
        
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialMeasurements;

        int frameId = 0;

        std::set<int> mapIds;

        for (const auto &frame : map->GetFrames())
        {
            const Eigen::Matrix4d position = frame.GetPosition().cast<double>();
            initialMeasurements.insert(gtsam::Symbol('x', frameId), gtsam::Pose3(position));
            
            // std::cout << frameId << "] ==================================================================================================================\n";

            if (frameId > 0)
            {
                const auto noise = CreateNoise6(0.2, 1.5);  

                // const Eigen::Matrix4d odom = frame.GetOdometry().cast<double>();
                // graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('x', frameId), gtsam::Symbol('x', frameId - 1), 
                //     gtsam::Pose3(odom), noise);
                graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise);
            }
            else
            {
                const auto noise = CreateNoise6(0.000001, 0.0000001); 

                graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise);
            }

            for (const auto &point : frame.GetPointsData())
            {
                if (point.associatedMapPointId == 0 || map->GetPoint(point.associatedMapPointId).observers < 2)
                    continue;

                mapIds.insert(point.associatedMapPointId);

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
                const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.5f);
                const auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.1), 
                    measurementNoise);
                
                graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
                    measurement2, huber, gtsam::Symbol('x', frameId), gtsam::Symbol('l', point.associatedMapPointId), this->cameraMat);

            }

            ++frameId;
        }
        
        // map
        unsigned mapPoints = 0;
        for (const auto pointId : mapIds)
        {
            const auto point = map->GetPoint(pointId);
            const auto mapPointGTSAM = gtsam::Point3( point.position.x, point.position.y, point.position.z );
            initialMeasurements.insert(gtsam::Symbol('l', pointId), mapPointGTSAM);

            // add uncertatinty to the map points
            const auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.25);
            graph.addPrior(gtsam::Symbol('l', pointId), mapPointGTSAM, priorNoise);

            ++mapPoints;
        }

        std::unique_ptr<gtsam::NonlinearOptimizer> optimizer;

        gtsam::LevenbergMarquardtParams params;
        params.lambdaFactor = 10;
        params.maxIterations = 1000;
        params.absoluteErrorTol = 0.00001;
        optimizer = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph, initialMeasurements, params);

        gtsam::Values optimizationResult = optimizer->optimize();

        // result.print("Optimization result:\n");
        // std::cout << "final error = " << graph.error(result) << std::endl;
        std::cout << "frames: " << frameId << ", mapPoints: " << mapPoints << std::endl;
        std::cout << "++initial error = " << graph.error(initialMeasurements) << std::endl;
        std::cout << "++final error = " << graph.error(optimizationResult) << std::endl << std::endl;

        for (int i = 0; i < frameId; ++i)
        {
            const gtsam::Pose3 targetFrame = optimizationResult.at<gtsam::Pose3>(gtsam::Symbol('x', i));
            Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
            result.block<3, 3>(0, 0) = targetFrame.rotation().matrix();
            result.block<3, 1>(0, 3) = targetFrame.translation();

            // std::cout << "ID: " << i << ", BEFORE:\n" << map->GetFrame(i).GetPosition() << std::endl;
            map->GetFrame(i).SetPosition(result.cast<float>());
            // std::cout << "AFTER:\n" << map->GetFrame(i).GetPosition() << std::endl;
        }

        for (const auto id : mapIds)
        {
            const gtsam::Point3 mapPoint = optimizationResult.at<gtsam::Point3>(gtsam::Symbol('l', id));
            cv::Point3f result { 
                    static_cast<float>(mapPoint.x()), 
                    static_cast<float>(mapPoint.y()), 
                    static_cast<float>(mapPoint.z())
                };

            // std::cout << "ID: " << id << ", BEFORE: " << map->GetPoint(id).position;
            map->GetPoint(id).position = result;
            // std::cout << ", AFTER: " << map->GetPoint(id).position << std::endl;
        }

        return true;
    }
} // namespace eacham
