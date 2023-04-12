#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "odometry/IFrameToMapOdometry.h"
#include "types/DataTypes.h"

namespace eacham
{
template<typename T>
class FrameToMapOdometry : public IFrameToMapOdometry<T>
{
public:
    bool Process(const T &data) override;
};

} // namespace eacham


namespace eacham
{

template<typename T>
bool FrameToMapOdometry<T>::Process(const T &data)
{
    Frame frame = this->frameCreator->Create(data);

    if (frame.isValid())
    {
        if (this->localMap->GetSize() > 0)
        {
            const auto& latestFrame = this->localMap->GetLatestFrame();
            const auto [odom, inliers] = this->motionEstimator->Estimate(latestFrame, frame);

            if (inliers == 0)
            {
                std::cout << "\n++++++++++\nMotion estimation error\n++++++++++\n";

                return false;
            }
            this->odometry = odom;

            this->SetPosition(latestFrame.GetPosition() * this->odometry);
        }

        frame.SetOdometry(this->odometry);
        frame.SetPosition(this->position);

        this->localMap->AddFrame(frame);

        if (this->localOptimizer != nullptr)
        {
            // this->localOptimizer->Optimize(this->localMap.get());

            this->SetPosition(this->localMap->GetLatestFrame().GetPosition());
        }
    }
    else
    {
        std::cout << "\n++++++++++\nMotion estimation error: Invalid frame\n++++++++++\n";

        return false;
    }

    return true;
}

}