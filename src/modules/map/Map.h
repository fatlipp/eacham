#pragma once

#include "map/MapPoint.h"
#include "map/MapFrame.h"
#include "frame/IFrame.h"
#include "motion_estimator/EstimationResult.h"

#include <list>
#include <vector>

namespace eacham
{

class Map
{

public:
    size_t GetSize() const
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        return frames.size();
    }

    std::list<MapFrame> GetFrames() const
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        return frames;
    }

    MapFrame& GetFrame(const unsigned id)
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        // auto firstItem = frames.begin();
        // std::advance(firstItem, id);

        for (auto& frame : this->frames)
        {
            if (frame.id == id)
            {
                return frame;
            }
        }

        std::cout << "id not found\n";
        
        static MapFrame f(0, {});
        return f;
    }

    MapPoint& GetPoint(const size_t id)
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        if (id == 0)
        {
            std::cerr << "Map Id must be greater than zero!\n";
        }

        return points[id - 1];
    }

    auto GetPoints() const
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        return points;
    }

    void lock()
    {
        this->mapMutex.lock();
    }

    void unlock()
    {
        this->mapMutex.unlock();
    }

public:
    bool AddFrame(const std::vector<FramePointData>& frameData, const EstimationResult& estimation);

    void Reset();

protected:
    void AddMapPoint(MapFrame& frame, const FramePointData& framePoint);

protected:
    std::list<MapFrame> frames;
    std::vector<MapPoint> points;

    mutable std::mutex globalMutex;
    mutable std::mutex mapMutex;
};

}