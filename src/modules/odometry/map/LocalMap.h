#pragma once

#include "odometry/frame/Frame.h"
#include <list>
#include <vector>


namespace odometry
{

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

    bool Optimize();

    size_t size() const
    {
        return frames.size();
    }

    std::list<Frame>& GetFrames() 
    {
        return frames;
    }

    std::vector<FramePoint3d>& GetPoints() 
    {
        return points3d;
    }

    FramePoint3d& GetPoint(unsigned id)
    {
        return points3d[id];
    }

private:
    unsigned capaticy;

    std::list<Frame> frames;
    std::vector<FramePoint3d> points3d;
};

}