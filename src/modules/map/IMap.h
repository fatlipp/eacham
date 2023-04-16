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

    // const std::list<Frame>& GetFrames() const
    // {
    //     return frames;
    // }

    std::list<Frame> GetFrames() const
    {
        std::lock_guard<std::mutex> lock(this->framesMutex);
        return frames;
    }

    const std::vector<MapPoint>& GetPoints() const
    {
        return points;
    }

    MapPoint& GetPoint(const size_t id)
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
        this->frames.push_back(frame);
    }

    virtual void Reset()
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        this->frames.clear();
        this->points.clear();
    }

protected:
    std::list<Frame> frames;
    std::vector<MapPoint> points;

    mutable std::mutex globalMutex;
    mutable std::mutex framesMutex;
};

}