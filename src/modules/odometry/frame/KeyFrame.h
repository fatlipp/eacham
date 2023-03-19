#pragma once

#include "odometry/frame/Frame.h"
#include "types/DataTypes.h"

namespace eacham
{

class KeyFrame : public Frame
{
public:
    KeyFrame()
        : Frame(-1.0, {})
    {
    }
};

} // namespace eacham
