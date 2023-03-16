#include "odometry/map/LocalMap.h"
#include "tools/Tools3d.h"

#include <eigen3/Eigen/Geometry>

namespace odometry
{
    void LocalMap::AddFrame(Frame &frame)
    {
        if (frames.size() > capaticy)
        {
            frames.pop_front();
        }

        static unsigned ID = 1;
        Eigen::Matrix4f framePos = frame.GetPosition();
        // Eigen::Affine3f aff = Eigen::Affine3f(framePos);

        for (auto &point : frame.GetPointsData())
        {
            if (point.point3d.mapPointId > 0)
            {
                GetPoint(point.point3d.mapPointId).observers++;

                continue;
            }

            point.point3d.SetMapPointId(ID++);

            auto ppp = point.point3d;
            ppp.position = tools::transformPoint3d(ppp.position, framePos);
            ppp.observers = 1;
            points3d.push_back(ppp);
        }

        frames.push_back(frame);
    }
}