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
#include "../visualization/Render.h"
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

        this->transform = Eigen::Affine3f::Identity();

        this->localMap = LocalMap(10U);
    }

    void SetCameraMatrix()
    {
        
    }

    std::vector<FramePoint3d> GetLocalMapPoints() 
    {
        return localMap.GetPoints();
    }

    bool Proceed(const T &data) override;

    Eigen::Matrix4f GetOdometry() override;

private:
    const IDataSourceCamera<T>* camera;

    LocalMap localMap;
    Frame lastFrame;
    std::unique_ptr<FrameCreator> frameCreator;
    std::unique_ptr<IMotionEstimator> motionEstimator;
    Eigen::Affine3f transform;

    LocalFramesOptimizer localOptimizer;
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
        Eigen::Matrix4f odom;

        if (lastFrame.isValid())
        {
            auto [odom, inliers] = motionEstimator->Estimate(lastFrame, frame);

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

        localMap.AddFrame(frame);

        // if (localMap.size() == 2)
        //     localOptimizer.Optimize(localMap);

        this->lastFrame = frame;
    }

    return true;
}

template<typename T>
Eigen::Matrix4f VisualOdometry<T>::GetOdometry()
{
    return this->lastFrame.GetPosition();
}

}