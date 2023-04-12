#pragma once

#include "data_source/IDataSource.h"
#include "odometry/IOdometry.h"
#include "visualization/IRender.h"
#include "config/Config.h"
#include "performance/BlockTimer.h"

#include <thread>

namespace eacham
{

template<typename T>
class Pipeline
{
public:
    Pipeline(const ConfigGeneral& config)
        : frameId(0)
        , maxFrames(config.maxFrames)
        , isPlay(false)
        , isStep(false)
        , isRunning(false)
    {
    }

    virtual ~Pipeline() = default;
public:
    bool IsActive() const
    {
        return this->isRunning;
    }

public:
    virtual void SetDataSource(std::unique_ptr<IDataSource<T>> dataSourceInp)
    {
        this->dataSource = std::move(dataSourceInp);
    }

    void SetOdometry(std::unique_ptr<IOdometry<T>> odometryInp)
    {
        this->odometry = std::move(odometryInp);
    }

    void SetRender(std::unique_ptr<IRender> renderInp)
    {
        this->render = std::move(renderInp);
    }

    void SetOnProcessComplete(std::function<void()> onProcessComplete)
    {
        this->onProcessComplete = onProcessComplete;
    }

// pipe
public:
    void Start()
    {
        this->render->Start();

        this->isRunning = true;
        this->pipelineThread = std::async(std::launch::async, &Pipeline::Loop, this);

    }

    void Kill()
    {
        this->isRunning = false;

        this->render->Stop();
    }

// actions
public:
    void Play()
    {
        this->isPlay = true;
        this->isStep = false;
    }

    void Pause()
    {
        this->isPlay = false;
        this->isStep = false;
    }

    void Step()
    {
        this->isPlay = true;
        this->isStep = true;
    }

protected:
    virtual bool Process()
    {
        std::cout << "frameId: " << this->frameId << std::endl;

        this->odometry->Process(this->dataSource->Get());

        ++this->frameId;

        return true;
    }

private:
    void Loop()
    {
        while (this->isRunning)
        {
            if (this->frameId < this->maxFrames)
            {
                if (this->isPlay)
                {
                    Process();
                }
                
                if (this->isStep)
                {
                    Pause();
                }
            }
        }
    }

protected:
    std::unique_ptr<IDataSource<T>> dataSource;
    std::unique_ptr<IOdometry<T>> odometry;
    std::unique_ptr<IRender> render;

    std::future<void> pipelineThread;
    std::atomic<bool> isRunning;

    unsigned frameId;
    unsigned maxFrames;

    std::atomic<bool> isPlay;
    std::atomic<bool> isStep;

    std::function<void()> onProcessComplete;
};
    
} // namespace eacham
