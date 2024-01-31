#pragma once

#include <pangolin/display/view.h>

namespace eacham
{
    
class IDrawable
{
public:
    IDrawable() = default;

public:
    virtual void Draw(pangolin::OpenGlRenderState& state) = 0;
    
};

}