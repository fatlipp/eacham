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
    size_t GetSize() const
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        return frames.size();
    }

    std::list<IFrame> GetFrames() const
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        return frames;
    }

    IFrame& GetFrame(const unsigned id)
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        // auto firstItem = frames.begin();
        // std::advance(firstItem, id);

        for (auto& frame : this->frames)
        {
            if (frame.GetId() == id)
            {
                return frame;
            }
        }

        std::cout << "id not found\n";
        
        IFrame f;
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

    void lock()
    {
        this->globalMutex.lock();
    }


    void unlock()
    {
        this->globalMutex.unlock();
    }


public:
    virtual void AddFrame(IFrame &frame)
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);
        this->frames.push_back(frame);
    }

    virtual void DeleteFrame(const size_t id)
    {
        std::lock_guard<std::mutex> lock(this->globalMutex);

        if (id == 0 && this->frames.size() > 0)
        {
            this->frames.pop_front();
        }
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
};

}