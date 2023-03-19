#include "LocalFramesOptimizer.h"

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
    LocalFramesOptimizer::LocalFramesOptimizer(const cv::Mat &cameraMat, const cv::Mat &distCoeffs)
    {
        if (cameraMat.rows == 1)
        {
            this->K = boost::make_shared<gtsam::Cal3_S2>(cameraMat.at<float>(0, 0), cameraMat.at<float>(0, 1), 
                                                        0.0, 
                                                        cameraMat.at<float>(0, 2) , cameraMat.at<float>(0, 3));
        }
        else
        {
            std::cerr << "NEED CHECK VALUES!!!!!!!!!\n";
        }
    }

    gtsam::noiseModel::Diagonal::shared_ptr CreateNoise6(const float posNoise, const float rot)
    {
        const float rotNoise = rot * 3.141592f / 180.0f;
        return gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(6) << gtsam::Vector3::Constant(rotNoise), gtsam::Vector3::Constant(posNoise)).finished());  
    }

    bool LocalFramesOptimizer::Optimize(LocalMap &map)
    {
        std::cout << "LocalFramesOptimizer() map.size() = " << map.size() << std::endl;

        if (map.size() < 5)
        {
            return false;
        }
        
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialMeasurements;

        const int INITIAL_ID = 0;
        int frameId = INITIAL_ID;
        int landmarkId = INITIAL_ID;

        for (auto &frame : map.GetFrames())
        {
            const Eigen::Matrix4d position = Eigen::Matrix4d::Identity();//frame.GetPosition().cast<double>();
            initialMeasurements.insert(gtsam::Symbol('x', frameId), gtsam::Pose3(position));
            
            std::cout << frameId << "] ==================================================================================================================\n";

            if (frameId > INITIAL_ID)
            {
                const auto noise = CreateNoise6(0.32, 3.5);  

                Eigen::Matrix4d odom = frame.GetOdometry().cast<double>();
                std::cout << frameId << ") odom:\n" << odom << std::endl;

                // graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('x', frameId), gtsam::Symbol('x', frameId - 1), 
                //     gtsam::Pose3(odom), noise);
                // graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise);
            }
            else
            {
                const auto noise = CreateNoise6(0.0001, 0.0001); 

                graph.addPrior(gtsam::Symbol('x', frameId), gtsam::Pose3(position), noise);
            }

            for (auto &point : frame.GetPointsData())
            {
                if (point.associatedMapPointId == 0 || map.GetPoint(point.associatedMapPointId).observers < 2)
                    continue;

                // const auto mapPoint = map.GetPoint(point.point3d.mapPointId).position;
                // const auto mapPointGTSAM = gtsam::Point3( mapPoint.x, mapPoint.y, mapPoint.z );
                // gtsam::PinholeCamera<gtsam::Cal3_S2> camera(gtsam::Pose3(position), *K);
                // const gtsam::Point2 measurement0 = camera.project2(mapPointGTSAM);
                // const gtsam::Point2 measurement1 = camera.projectSafe(mapPointGTSAM).first;
                const gtsam::Point2 measurement2 = {point.keypoint.pt.x, point.keypoint.pt.y};

                // std::cout << "measurement = 1: [" << measurement0.x() << ", " << measurement0.y() << 
                //     "], 2: [" << measurement1.x() << ", " << measurement1.y() << 
                //     "], 3: [" << measurement2.x() << ", " << measurement2.y() << 
                //     "], map: [" << mapPointGTSAM.x() << ", " << mapPointGTSAM.y() << ", " << mapPointGTSAM.z() << "] " << std::endl;

                const auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.1);
                graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
                    measurement2, measurementNoise, gtsam::Symbol('x', frameId), gtsam::Symbol('l', point.associatedMapPointId), this->K);

            }

            ++frameId;
        }
        
        // map
        unsigned mapPoints = 0;
        for (auto &point : map.GetPoints())
        {
            if (point.id == 0 || point.observers < 2)
                continue;

            ++mapPoints;

            const auto mapPointGTSAM = gtsam::Point3( point.position.x, point.position.y, point.position.z );
            initialMeasurements.insert(gtsam::Symbol('l', point.id), mapPointGTSAM);

            // make const
            // graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Point3> >(gtsam::Symbol('l', point.mapPointId), mapPointGTSAM);

            // add uncertatinty to the map points
            const auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.15);
            graph.addPrior(gtsam::Symbol('l', point.id), mapPointGTSAM, priorNoise);
        }


        std::unique_ptr<gtsam::NonlinearOptimizer> optimizer;

        gtsam::GaussNewtonParams parameters;
        parameters.relativeErrorTol = 1e-7;
        parameters.maxIterations = 10000;
        // optimizer = std::make_unique<gtsam::GaussNewtonOptimizer>(graph, initialMeasurements, parameters);

        gtsam::LevenbergMarquardtParams params;
        params.lambdaInitial = 1;
        params.lambdaFactor = 10;
        params.maxIterations = 1000;
        params.absoluteErrorTol = 0.000000000001;
        params.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
        params.verbosityLM = gtsam::LevenbergMarquardtParams::TRYLAMBDA;
        params.linearSolverType = gtsam::NonlinearOptimizerParams::CHOLMOD;
        // optimizer = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph, initialMeasurements, params);
        optimizer = std::make_unique<gtsam::DoglegOptimizer>(graph, initialMeasurements);

        // std::cout << "Before" << std::endl;

        // for (int i = 1; i < frameId; ++i)
        // {
        //     gtsam::Pose3 frame1 = initialMeasurements.at<gtsam::Pose3>(gtsam::Symbol('x', i));
        //     std::cout << "frame " << i << " = " << frame1 << std::endl;
        // }

        // for (int i = 1; i < 5; ++i)
        // {
        //     gtsam::Point3 frame1 = initialMeasurements.at<gtsam::Point3>(gtsam::Symbol('l', mapIds[i]));
        //     std::cout << "mapPoint " << mapIds[i] << " = [" << frame1.x() << ", " << frame1.y() << ", " << frame1.z() << "]" << std::endl;
        // }

        gtsam::Values result = optimizer->optimize();

        // result.print("Optimization result:\n");
        // std::cout << "final error = " << graph.error(result) << std::endl;
        std::cout << "frames: " << frameId << ", mapPoints: " << mapPoints << std::endl;
        std::cout << "++initial error = " << graph.error(initialMeasurements) << std::endl;
        std::cout << "++final error = " << graph.error(result) << std::endl << std::endl;

        for (int i = 1; i < frameId; ++i)
        {
            gtsam::Pose3 frame1 = result.at<gtsam::Pose3>(gtsam::Symbol('x', i));
            std::cout << "frame " << i << " = " << frame1 << std::endl;
        }

        // for (int i = 1; i < 5; ++i)
        // {
        //     gtsam::Point3 frame1 = result.at<gtsam::Point3>(gtsam::Symbol('l', mapIds[i]));
        //     std::cout << "mapPoint " << mapIds[i] << " = [" << frame1.x() << ", " << frame1.y() << ", " << frame1.z() << "]" << std::endl;
        // }

        return true;
    }
} // namespace eacham
