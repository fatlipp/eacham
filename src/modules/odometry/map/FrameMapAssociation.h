#pragma once

namespace eacham
{

struct FrameMapAssociation
{
    unsigned mapId;
    unsigned frameId;
    unsigned framePointId;

    FrameMapAssociation(const unsigned mapId, const unsigned frameId, const unsigned framePointId)
        : mapId(mapId)
        , frameId(frameId)
        , framePointId(framePointId)
        {}
};

}