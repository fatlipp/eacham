#pragma once

#include <vector>
#include <list>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "odometry/IFrameToFrameOdometry.h"
#include "types/DataTypes.h"

namespace eacham
{
template<typename T>
class FrameToFrameOdometry : public IFrameToFrameOdometry<T>
{
public:
    bool Process(const T &data) override;
};

} // namespace eacham


namespace eacham
{

template<typename T>
bool FrameToFrameOdometry<T>::Process(const T &data)
{
    Frame frame = this->frameCreator->Create(data);

    if (frame.isValid())
    {
        if (this->lastFrame.isValid())
        {
            const auto [odom, inliers] = this->motionEstimator->Estimate(this->lastFrame, frame);

            if (inliers == 0)
            {
                std::cout << "\n++++++++++\nMotion estimation error\n++++++++++\n";

                return false;
            }
            this->odometry = odom;

            this->SetPosition(this->lastFrame.GetPosition() * this->odometry);
        }

        frame.SetOdometry(this->odometry);
        frame.SetPosition(this->position);
        
        std::lock_guard<std::mutex> lock(this->syncMutex);
        this->lastFrame = frame;
    }
    else
    {
        std::cout << "\n++++++++++\nMotion estimation error: Invalid frame\n++++++++++\n";

        return false;
    }

    return true;
}

}