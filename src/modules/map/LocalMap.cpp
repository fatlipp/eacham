#include "map/LocalMap.h"
#include "tools/Tools3d.h"

#include <eigen3/Eigen/Geometry>

namespace eacham
{
    void LocalMap::AddFrame(const Frame &frame)
    {
        // static unsigned ID = 1;
        // const Eigen::Matrix4f framePos = frame.GetPosition();

        // for (const auto &point : frame.GetPointsData())
        // {
        //     if (point.associatedMapPointId > 0)
        //     {
        //         GetPoint(point.associatedMapPointId).AddObserver();

        //         continue;
        //     }

        //     MapPoint mapPoint { ID++, transformPoint3d(point.position3d, framePos) };
        //     mapPoint.observers = 1;
        //     this->points.push_back(mapPoint);

        //     // point.associatedMapPointId = mapPoint.id;
        // }

        // this->frames.push_back(frame);

        // if (this->GetSize() == this->capaticy)
        // {
        //     this->frames.pop_front();
        // }
    }
}