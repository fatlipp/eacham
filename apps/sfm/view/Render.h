#pragma once

#include "visualization/render/IRender.h"

#include <thread>
#include <future>

#include <Eigen/Core>

namespace eacham
{

class Render : public IRender
{

public:
    void Activate() override;
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

    void SetOnBAClick(std::function<void()> onBAClick)
    {
        this->onBAClick = onBAClick;
    }

    void SetOnResetClick(std::function<void()> onResetClick)
    {
        this->onResetClick = onResetClick;
    }

    void SetOnCloseClick(std::function<void()> onCloseClick)
    {
        this->onCloseClick = onCloseClick;
    }

private:
    void Loop();

private:
    std::future<void> drawThread;
    std::atomic<bool> isRunning;
    std::mutex mute;

    std::function<void()> onPlayClick;
    std::function<void()> onStepClick;
    std::function<void()> onBAClick;
    std::function<void()> onResetClick;
    std::function<void()> onCloseClick;
};

}