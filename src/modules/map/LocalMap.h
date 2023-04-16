#pragma once

#include "map/IMap.h"
#include "map/MapPoint.h"
#include "frame/Frame.h"


namespace eacham
{

class LocalMap : public IMap
{
public:
    LocalMap()
        : capaticy(5U) 
        {}

    LocalMap(const unsigned capaticy)
        : capaticy(capaticy) 
        {}

public:
    void AddFrame(const Frame &frame) override;

private:
    unsigned capaticy;
};

}