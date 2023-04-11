#pragma once

#include "visualization/IRender.h"

#include <thread>
#include <future>

#include <eigen3/Eigen/Core>

namespace eacham
{

class Render : public IRender
{
public:
    Render();

public:
    void Start() override;
    void Stop() override;

public:
    bool IsActive() const override
    {
        return this->isRunning;
    }

public:
    void SetOnPlayClick(std::function<void()> onPlayClick)
    {
        this->onPlayClick = onPlayClick;
    }

    void SetOnStepClick(std::function<void()> onStepClick)
    {
        this->onStepClick = onStepClick;
    }

    void SetOnCloseClick(std::function<void()> onCloseClick)
    {
        this->onCloseClick = onCloseClick;
    }

private:
    std::future<void> drawThread;
    std::atomic<bool> isRunning;
    std::mutex mute;

    std::function<void()> onPlayClick;
    std::function<void()> onStepClick;
    std::function<void()> onCloseClick;
};

}