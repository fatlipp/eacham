#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "odometry/IFrameToMapOdometry.h"
#include "frame/IFrameCreator.h"
#include "motion_estimator/IMotionEstimator.h"
#include "optimization/LocalFramesOptimizer.h"
#include "types/DataTypes.h"

namespace eacham
{
template<typename T>
class VisualOdometry : public IFrameToMapOdometry<T>
{
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

private:
    std::unique_ptr<IFrameCreator> frameCreator;
    std::unique_ptr<IMotionEstimator> motionEstimator;
    std::unique_ptr<LocalFramesOptimizer> localOptimizer;

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
            frame.SetPosition(Eigen::Matrix4f::Identity());
        }

        this->localMap->AddFrame(frame);

        if (localOptimizer != nullptr)
        {
            localOptimizer->Optimize(this->localMap.get());
        }

        this->lastFrame = this->localMap->GetLatest();
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