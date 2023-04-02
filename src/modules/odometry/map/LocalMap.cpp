#include "odometry/map/LocalMap.h"
#include "tools/Tools3d.h"

#include <eigen3/Eigen/Geometry>

namespace eacham
{
    void LocalMap::AddFrame(Frame &frame)
    {
        if (GetSize() >= capaticy)
        {
            frames.pop_front();
        }

        static unsigned ID = 1;
        const Eigen::Matrix4f framePos = frame.GetPosition();

        for (auto &point : frame.GetPointsData())
        {
            if (point.associatedMapPointId > 0)
            {
                GetPoint(point.associatedMapPointId).AddObserver();

                continue;
            }

            MapPoint3d mapPoint { ID++, transformPoint3d(point.position3d, framePos) };
            mapPoint.observers = 1;
            points3d.push_back(mapPoint);

            point.associatedMapPointId = mapPoint.id;
        }

        static unsigned FRAME_ID = 1;
        frame.id = FRAME_ID++;
        
        frames.push_back(frame);
    }
}