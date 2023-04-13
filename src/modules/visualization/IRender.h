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

    virtual void Start() = 0;
    virtual void Stop() = 0;

public:
    virtual bool IsActive() const = 0;

public:
    void Add(std::unique_ptr<IDrawable> drawable)
    {
        this->drawables.emplace_back(std::move(drawable));
    }

    void Draw()
    {
        for (auto& drawable : drawables)
        {
            drawable->Draw();
        }    
    }

protected:
    std::vector<std::unique_ptr<IDrawable>> drawables;
};

}