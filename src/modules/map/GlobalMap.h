#pragma once

#include "map/IMap.h"
#include "map/MapPoint.h"
#include "frame/IFrame.h"


namespace eacham
{

class GlobalMap : public IMap
{
public:
    GlobalMap(const unsigned capacityInp)
        : capaticy(capacityInp) 
        {}

public:
    void AddFrame(IFrame &frame) override;

private:
    const unsigned capaticy;
};

}