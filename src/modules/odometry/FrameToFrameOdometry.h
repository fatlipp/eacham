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

    if (this->map->GetSize() > 0)
    {
        std::lock_guard<std::mutex> lock(this->syncMutex);
        this->lastFrame = this->map->GetFrames().back();

        std::cout << "GOT FRAME" << std::endl;
        std::cout << "GOT FRAME" << std::endl;
        std::cout << "GOT FRAME" << std::endl;
        std::cout << "GOT FRAME" << std::endl;
        std::cout << "GOT FRAME11111111111111111111111111111111111" << std::endl;
    }

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
        this->map->AddFrame(this->lastFrame);
    }
    else
    {
        std::cout << "\n++++++++++\nMotion estimation error: Invalid frame\n++++++++++\n";

        return false;
    }

    return true;
}

}