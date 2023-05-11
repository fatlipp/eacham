#pragma once

#include <future>
#include <atomic>
#include <thread>
#include <chrono>

namespace eacham
{

class IPipeline
{
public:
    IPipeline()
        : isActive(false)
        {}
        
public:
    virtual void Activate()
    {
        this->isActive = true;
        this->pipelineThread = std::async(std::launch::async, &IPipeline::Loop, this);
    }

    virtual void Deactivate()
    {
        if (this->pipelineThread.valid())
        {
            this->isActive = false;
            this->pipelineThread.get();
        }
    }

    bool IsActive()
    {
        return isActive;
    }

protected:
    void Loop()
    {
        while (this->isActive)
        {
            Process();
        }
    }

    virtual void Process() = 0;

protected:
    std::future<void> pipelineThread;
    std::atomic<bool> isActive;
};

}