#pragma once

#include "odometry/frame/Frame.h"
#include <list>
#include <vector>


namespace eacham
{

struct MapPoint3d
{
    unsigned id = 0;
    unsigned observers = 0;
    cv::Point3f position;

    MapPoint3d(const unsigned id, const cv::Point3f &position)
        : id(id)
        , position(position)
        {}

    void AddObserver() noexcept
    {
        ++observers;
    }
};

class LocalMap
{
public:
    LocalMap()
        : capaticy(5U) 
        {}

    LocalMap(const unsigned capaticy)
        : capaticy(capaticy) 
        {}

    void AddFrame(Frame &frame);

    Frame GetLatest() const
    {
        return frames.back();
    }

    size_t GetSize() const
    {
        return frames.size();
    }

    int GetCapacity() const
    {
        return capaticy;
    }

    std::list<Frame>& GetFrames() 
    {
        return frames;
    }

    Frame& GetFrame(const size_t id) 
    {
        auto firstItem = frames.begin();
        std::advance(firstItem, id);

        return *firstItem;
    }

    std::vector<MapPoint3d>& GetPoints() 
    {
        return points3d;
    }

    MapPoint3d& GetPoint(unsigned id)
    {
        return points3d[id - 1];
    }

private:
    unsigned capaticy;

    std::list<Frame> frames;
    std::vector<MapPoint3d> points3d;
};

}