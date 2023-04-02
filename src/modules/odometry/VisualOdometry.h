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
#include "odometry/frame/IFrameCreator.h"
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
    VisualOdometry(std::unique_ptr<IFrameCreator> frameCreator,
                         const FeatureExtractorType &featureExtractorType, 
                         const MotionEstimatorType &motionEstimatorType, 
                         const IDataSourceCamera<T>* camera)
        : frameCreator(std::move(frameCreator))
        , camera(camera)
    {
        switch (motionEstimatorType)
        {
        case MotionEstimatorType::OPT:
            this->motionEstimator = std::make_unique<MotionEstimatorOpt>(featureExtractorType, camera->GetParameters(), camera->GetDistortion());
            break;
        
        default:
            this->motionEstimator = std::make_unique<MotionEstimatorPnP>(featureExtractorType, camera->GetParameters(), camera->GetDistortion());
            break;
        }

        this->localOptimizer = std::make_unique<LocalFramesOptimizer>(camera->GetParameters(), camera->GetDistortion());
        this->localMap = std::make_unique<LocalMap>(10U);
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

    const Frame& GetCurrentFrame() const 
    {
        return this->lastFrame;
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
    std::unique_ptr<IFrameCreator> frameCreator;
    std::unique_ptr<IMotionEstimator> motionEstimator;
    std::unique_ptr<LocalFramesOptimizer> localOptimizer;
};

} // namespace eacham


namespace eacham
{

template<typename T>
bool VisualOdometry<T>::Proceed(const T &data)
{
    Frame frame = frameCreator->Create(data, camera->GetParameters());

    if (frame.isValid())
    {
        if (lastFrame.isValid())
        {
             auto [odom, inliers] = motionEstimator->Estimate(lastFrame, frame);

            if (inliers == 0)
            {
                std::cout << "\n++++++++++\nMotion estimation error\n++++++++++\n";

                return false;
            }

            // odom(0, 3) = -0.04f;
            // odom(1, 3) = -0.02f;
            // odom(2, 3) = 0.85f;
            
            frame.SetOdometry(odom);
            frame.SetPosition(lastFrame.GetPosition() * odom);

            if (inliers < 20)
            {
            }
        }
        else
        {
            Eigen::Matrix4f initialPos = Eigen::Matrix4f::Identity();
            initialPos << -0.42516577243804932, -0.58992290496826172, 0.68641793727874756, -2.550800085067749,
            -0.90508449077606201,     0.27779990434646606,    -0.32190239429473877,     0.98720002174377441,
            -0.00077596306800842285 ,   -0.75814914703369141,    -0.65203142166137695,      1.1018999814987183,
            0,                       0,                       0,                       1;

            frame.SetOdometry(Eigen::Matrix4f::Identity());
            frame.SetPosition(initialPos);
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
    else
    {
        std::cout << "\n++++++++++\nMotion estimation error: Invalid frame\n++++++++++\n";

        return false;
    }

    return true;
}

template<typename T>
Eigen::Matrix4f VisualOdometry<T>::GetOdometry()
{
    return this->lastFrame.GetPosition();
}

}