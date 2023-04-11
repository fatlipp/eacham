#pragma once

#include "map/MapPoint.h"
#include "frame/Frame.h"
#include <list>
#include <vector>


namespace eacham
{

class IMap
{
public:
    IMap() = default;

public:
    size_t GetSize() const
    {
        return frames.size();
    }

    const Frame& GetLatestFrame() const
    {
        return frames.back();
    }

    const Frame& GetFrame(const size_t id) const
    {
        auto firstItem = frames.begin();
        std::advance(firstItem, id);

        return *firstItem;
    }

    const std::list<Frame>& GetFrames() const
    {
        return frames;
    }

    const std::vector<MapPoint>& GetPoints() const
    {
        return points;
    }

    const MapPoint& GetPoint(const size_t id) const
    {
        if (id == 0)
        {
            std::cerr << "Map Id must be greater than zero!\n";
        }

        return points[id - 1];
    }

public:
    virtual void AddFrame(const Frame &frame)
    {
        frames.push_back(frame);
    }

protected:
    std::list<Frame> frames;
    std::vector<MapPoint> points;
};

}