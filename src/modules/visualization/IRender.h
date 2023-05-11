#pragma once

#include <thread>
#include <future>

#include <eigen3/Eigen/Core>

#include "visualization/IDrawable.h"

namespace eacham
{
    
class IRender
{
public:
    ~IRender() = default;

    virtual void Activate() = 0;
    virtual void Stop() = 0;

public:
    virtual bool IsActive() const = 0;

public:
    void Add(std::unique_ptr<IDrawable> drawable)
    {
        this->drawables.emplace_back(std::move(drawable));
    }

    void Draw(pangolin::OpenGlRenderState& state)
    {
        for (auto& drawable : drawables)
        {
            drawable->Draw(state);
        }    
    }

protected:
    std::vector<std::unique_ptr<IDrawable>> drawables;
};

}