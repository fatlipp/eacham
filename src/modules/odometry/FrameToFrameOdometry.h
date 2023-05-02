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
    IFrame frame = this->frameCreator->Create(data);

    if (!frame.isValid())
    {
        std::cout << "\n++++++++++\nMotion estimation error: Invalid frame\n++++++++++\n";

        return false;
    }

    const auto lastFrame = IVisualOdometry<T>::GetLastFrame();
    
    if (lastFrame.isValid())
    {
        const auto [odom, inliers] = this->motionEstimator->Estimate(lastFrame, frame);

        if (inliers == 0)
        {
            std::cout << "\n++++++++++\nMotion estimation error\n++++++++++\n";

            return false;
        }

        this->WaitForLocalMap();

        const auto lastFrameNew = IVisualOdometry<T>::GetLastFrame().GetPosition();
        this->odometry = odom;
        this->position = (lastFrameNew * this->odometry);
    }

    frame.SetOdometry(this->odometry);
    frame.SetPosition(this->position);

    this->map->AddFrame(frame);

    return true;
}

}