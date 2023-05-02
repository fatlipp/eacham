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

    if (frame.isValid())
    {
        std::cout << "GetLastFrame: 1";
        const auto lastFrame = IVisualOdometry<T>::GetLastFrame();
        std::cout << "GetLastFrame: 2";
        
        if (lastFrame.isValid())
        {
            const auto [odom, inliers] = this->motionEstimator->Estimate(lastFrame, frame);

            if (inliers == 0)
            {
                std::cout << "\n++++++++++\nMotion estimation error\n++++++++++\n";

                return false;
            }

            this->odometry = odom;

            std::cout << "this->isMapOptimizationProcess start" << std::endl;
            while (this->isMapOptimizationProcess)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            std::cout << "this->isMapOptimizationProcess end" << std::endl;

            const auto lastFrameNew = IVisualOdometry<T>::GetLastFrame().GetPosition();
            this->SetPosition(lastFrameNew * this->odometry);
        }

        frame.SetOdometry(this->odometry);
        frame.SetPosition(this->position);
        
        std::cout << "this->isMapOptimizationProcess start 2" << std::endl;
        while (this->isMapOptimizationProcess)
        {
        }
        std::cout << "this->isMapOptimizationProcess end 2" << std::endl;
        this->map->AddFrame(frame);
        std::cout << "ODOM this->GetSize(): " << this->map->GetSize() << std::endl;
    }
    else
    {
        std::cout << "\n++++++++++\nMotion estimation error: Invalid frame\n++++++++++\n";

        return false;
    }

    return true;
}

}