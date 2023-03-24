#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "IOdometry.h"
#include "tools/Tools3d.h"
#include "types/DataTypes.h"
#include "odometry/map/LocalMap.h"
#include "data_source/IDataSourceCamera.h"
#include "odometry/frame/FrameCreator.h"
#include "odometry/frame/KeyFrame.h"
#include "odometry/optimization/LocalFramesOptimizer.h"
#include "motion_estimator/MotionEstimatorPnP.h"
#include "motion_estimator/MotionEstimatorOpt.h"
#include "motion_estimator/IMotionEstimator.h"
#include "odometry/motion_estimator/MotionEstimatorType.h"

namespace eacham
{
template<typename T>
class VisualOdometry : public IOdometry<T>
{
public:
    VisualOdometry(const FeatureExtractorType &featureExtractor, const MotionEstimatorType &type, const IDataSourceCamera<T>* camera)
        : camera(camera)
    {
        this->frameCreator = std::make_unique<FrameCreator>(featureExtractor);

        switch (type)
        {
        case MotionEstimatorType::OPT:
            this->motionEstimator = std::make_unique<MotionEstimatorOpt>(featureExtractor, camera->GetParameters(), camera->GetDistortion());
            break;
        
        default:
            this->motionEstimator = std::make_unique<MotionEstimatorPnP>(featureExtractor, camera->GetParameters(), camera->GetDistortion());
            break;
        }

        this->localOptimizer = std::make_unique<LocalFramesOptimizer>(camera->GetParameters(), camera->GetDistortion());
        this->localMap = std::make_unique<LocalMap>(3U);
    }

    void SetCameraMatrix()
    {
    }

    std::vector<MapPoint3d> GetLocalMapPoints() 
    {
        return localMap->GetPoints();
    }

    std::list<Frame> GetLocalMapFrames() 
    {
        return localMap->GetFrames();
    }

    void SetLocalOptimizerState(const bool state)
    {
        this->isLocalOptimizerEnabled = state;
    }

    bool Proceed(const T &data) override;

    Eigen::Matrix4f GetOdometry() override;

private:
    const IDataSourceCamera<T>* camera;

    bool isLocalOptimizerEnabled;

    Frame lastFrame;

    std::unique_ptr<LocalMap> localMap;
    std::unique_ptr<FrameCreator> frameCreator;
    std::unique_ptr<IMotionEstimator> motionEstimator;
    std::unique_ptr<LocalFramesOptimizer> localOptimizer;
};

} // namespace eacham


namespace eacham
{

template<typename T>
bool VisualOdometry<T>::Proceed(const T &data)
{
    std::cout << "----------------------------------------------------------------------------------------" << std::endl;

    Frame frame = frameCreator->Create(data, camera->GetParameters());

    if (frame.isValid())
    {
        if (lastFrame.isValid())
        {
            const auto [odom, inliers] = motionEstimator->Estimate(lastFrame, frame);

            if (inliers == 0)
            {
                std::cout << "\n++++++++++\nMotion estimation error\n++++++++++\n";

                return false;
            }
            
            frame.SetOdometry(odom);
            frame.SetPosition(lastFrame.GetPosition() * odom);
        }
        else
        {
            frame.SetOdometry(Eigen::Matrix4f::Identity());
            frame.SetPosition(Eigen::Matrix4f::Identity());
        }

        localMap->AddFrame(frame);

        if (this->isLocalOptimizerEnabled)
        {
            static int cc = 0;
            cc++;

            if (cc % 3 == 0)
            {
                cc = 0;
                localOptimizer->Optimize(localMap.get());
            }
        }

        this->lastFrame = localMap->GetLatest();
    }

    return true;
}

template<typename T>
Eigen::Matrix4f VisualOdometry<T>::GetOdometry()
{
    return this->lastFrame.GetPosition();
}

}