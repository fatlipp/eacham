#include "map/LocalMap.h"
#include "tools/Tools3d.h"

#include <eigen3/Eigen/Geometry>

#include <mutex>

namespace eacham
{
    void LocalMap::AddFrame(IFrame &frame)
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        
        static unsigned ID = 1;
        const Eigen::Matrix4f framePos = frame.GetPosition();

        for (auto &point : frame.GetPointsData())
        {
            if (point.associatedMapPointId > 0)
            {
                GetPoint(point.associatedMapPointId).AddObserver();

                continue;
            }

            MapPoint mapPoint { ID++, tools::transformPoint3d(point.position3d, framePos) };
            mapPoint.observers = 1;
            this->points.push_back(mapPoint);

            point.associatedMapPointId = mapPoint.id;
        }

        {
            std::lock_guard<std::mutex> lock(this->framesMutex);
            this->frames.push_back(frame);
        }

        if (this->GetSize() > this->capaticy)
        {
            std::lock_guard<std::mutex> lock(this->framesMutex);
            this->frames.pop_front();
        }
    }
}