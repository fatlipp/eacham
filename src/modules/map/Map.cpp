#include "map/Map.h"
#include "frame/IFrame.h"
#include "motion_estimator/EstimationResult.h"
#include "tools/Tools3d.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eacham
{

bool Map::AddFrame(const std::vector<FramePointData>& currentFrameData, const EstimationResult& estimation)
{
    if (estimation.matches.size() > 100)
    {
        // return true;
    }

    MapFrame mapFrame { .id = estimation.frameIdCurrent, 
                        .parentId = 0, 
                        .isOptimized = false,
                        .odometry = estimation.odometry, 
                        .position = Eigen::Matrix4f::Identity() };


    if (estimation.frameIdPrev > 0)
    {
        mapFrame.id = estimation.frameIdCurrent;
        mapFrame.parentId = estimation.frameIdPrev;

        const auto& prevFrame = GetFrame(estimation.frameIdPrev);

        if (!prevFrame.isValid())
        {
            std::cerr << "prevFrame is not found." << std::endl;

            return false;
        }

        this->lock();

        mapFrame.position = prevFrame.position * mapFrame.odometry;

        for (const auto& currentFramePoint : currentFrameData)
        {
            // a matched point
            if (estimation.matches.contains(currentFramePoint.id))
            {
                const unsigned idOnPrevFrame = estimation.matches.at(currentFramePoint.id);

                if (!prevFrame.pointsData.contains(idOnPrevFrame))
                {
                    std::cerr << "prevFrame doesn't contain needed point." << std::endl;

                    return false;
                }

                const auto prevPointData = prevFrame.pointsData.at(idOnPrevFrame);
                const auto mapPointId = prevPointData.mapPointId;

                mapFrame.AddPoint(currentFramePoint.id, { .mapPointId = mapPointId, .keypoint = currentFramePoint.keypoint });

                // current frame also observes this point
                GetPoint(mapPointId).observers++;
            }
            // a new point
            else
            {
                AddMapPoint(mapFrame, currentFramePoint);
            }
        }
    }
    else
    {
        this->lock();
        for (const auto& currentFramePoint : currentFrameData)
        {
            AddMapPoint(mapFrame, currentFramePoint);
        }
    }
    std::cout << "Full size: " << currentFrameData.size() << ", matches: " << estimation.matches.size() << std::endl;
    std::cout << "Added frame id: " << mapFrame.id << std::endl;

    std::lock_guard<std::mutex> lock(this->globalMutex);
    this->frames.push_back(mapFrame);
    this->unlock();

    return true;
}

void Map::Reset()
{
    std::lock_guard<std::mutex> lock(this->globalMutex);
    this->frames.clear();
    this->points.clear();
}

void Map::AddMapPoint(MapFrame& frame, const FramePointData& framePoint)
{
    static unsigned MAP_POINT_ID = 1;

    frame.AddPoint(framePoint.id, { .mapPointId = MAP_POINT_ID, .keypoint = framePoint.keypoint });

    {
        const auto pos = tools::transformPoint3d(framePoint.position3d, frame.position);
            // MapPoint mapPoint { ID++, tools::transformPoint3d(point.position3d, framePos) };
        MapPoint mapPoint { .id = MAP_POINT_ID, .position = pos, .observers = 1 };

        std::lock_guard<std::mutex> lock(this->globalMutex);
        this->points.push_back(mapPoint);
    }

    ++MAP_POINT_ID;
}

}