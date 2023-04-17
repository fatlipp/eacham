#pragma once

#include "map/MapPoint.h"
#include "frame/IFrame.h"
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

    std::list<IFrame> GetFrames() const
    {
        std::lock_guard<std::mutex> lock(this->framesMutex);
        return frames;
    }

    IFrame& GetFrame(const size_t id)
    {
        auto firstItem = frames.begin();
        std::advance(firstItem, id);

        return *firstItem;
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
    virtual void AddFrame(IFrame &frame)
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
    std::list<IFrame> frames;
    std::vector<MapPoint> points;

    mutable std::mutex globalMutex;
    mutable std::mutex framesMutex;
};

}