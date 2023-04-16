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

Frame GenerateFrameFromMap(IMap *map)
{
    Frame result = map->GetLatestFrame();

    for (const auto& frame : map->GetFrames())
    {
        result.AddPointDatas(frame.GetPointsData());
    }

    return result;
}

template<typename T>
bool FrameToMapOdometry<T>::Process(const T &data)
{
    Frame frame = this->frameCreator->Create(data);

    if (frame.isValid())
    {
        if (this->localMap->GetSize() > 0)
        {
            const auto latestFrame = GenerateFrameFromMap(this->localMap.get());
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
        
        {
            std::lock_guard<std::mutex> lock(this->syncMutex);
            this->localMap->AddFrame(frame);

            if (this->localOptimizer != nullptr)
            {
                // this->localOptimizer->Optimize(this->localMap.get());

                this->SetPosition(this->localMap->GetLatestFrame().GetPosition());
            }
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