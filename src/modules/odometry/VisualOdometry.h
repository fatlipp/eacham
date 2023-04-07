#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "IOdometry.h"
#include "types/DataTypes.h"
#include "data_source/IDataSourceCamera.h"
#include "frame/IFrameCreator.h"
#include "motion_estimator/IMotionEstimator.h"
#include "optimization/LocalFramesOptimizer.h"
#include "map/LocalMap.h"

namespace eacham
{
template<typename T>
class VisualOdometry : public IOdometry<T>
{
public:
    VisualOdometry()
    {
        this->localMap = std::make_unique<LocalMap>(10U);
    }

public:
    bool Proceed(const T &data) override;

    Eigen::Matrix4f GetOdometry() override;

public:
    void SetFrameCreator(std::unique_ptr<IFrameCreator> frameCreatorInp)
    {
        this->frameCreator = std::move(frameCreatorInp);
    }

    void SetMotionEstimator(std::unique_ptr<IMotionEstimator> motionEstimatorInp)
    {
        this->motionEstimator = std::move(motionEstimatorInp);
    }

    void SetLocalOptimizer(std::unique_ptr<LocalFramesOptimizer> optimizerInp)
    {
        this->localOptimizer = std::move(optimizerInp);
    }

    void SetDefaultPos(const Eigen::Matrix4f& pos)
    {
        this->defaultPos = pos;
    }

public:
    std::vector<MapPoint3d> GetLocalMapPoints() 
    {
        return localMap->GetPoints();
    }

    std::list<Frame> GetLocalMapFrames() 
    {
        return localMap->GetFrames();
    }

private:
    std::unique_ptr<IFrameCreator> frameCreator;
    std::unique_ptr<IMotionEstimator> motionEstimator;
    std::unique_ptr<LocalFramesOptimizer> localOptimizer;

    std::unique_ptr<LocalMap> localMap;

    Eigen::Matrix4f defaultPos;
    Frame lastFrame;
};

} // namespace eacham


namespace eacham
{

template<typename T>
bool VisualOdometry<T>::Proceed(const T &data)
{
    Frame frame = frameCreator->Create(data);

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
            frame.SetPosition(defaultPos);
        }

        localMap->AddFrame(frame);

        if (localOptimizer != nullptr)
        {
            localOptimizer->Optimize(localMap.get());
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