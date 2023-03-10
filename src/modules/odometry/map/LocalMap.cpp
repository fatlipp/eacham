#include "odometry/map/LocalMap.h"

namespace odometry
{
    void LocalMap::AddFrame(Frame &frame)
    {
        frames.push_back(frame);

        if (frames.size() > capaticy)
        {
            frames.pop_front();
        }

        static unsigned ID = 1;

        for (auto &point : frame.GetPointsData())
        {
            const unsigned mapPointId = ID++; 
            
            point.point3d.SetMapPointId(mapPointId);
            points3d.push_back(point.point3d);
        }
    }

    // bool LocalMap::Optimize()
    // {
    //     if (frames.size() == capaticy)
    //     {
    //         // optimize
    //         const bool isOptimized = localOptimizer.Optimize(*this);

    //         if (isOptimized)
    //         {
    //             std::cout << "Optimized.." << std::endl;
    //         }

    //         return true;
    //     }

    //     return false;
    // }

}