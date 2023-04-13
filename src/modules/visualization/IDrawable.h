#pragma once

namespace eacham
{
    
class IDrawable
{
public:
    IDrawable() = default;

public:
    virtual void Draw() = 0;
    
};

}