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
    {
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

public:
    void Start()
    {
        this->render->Start();

        this->isStarted = true;

        while (this->render->IsActive())
        {
            Loop();
        }
    }

    void Stop()
    {
        this->isStarted = false;
    }

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
        if (this->frameId >= this->maxFrames)
        {
            return false;
        }

        std::cout << "this->frameId: " << this->frameId << std::endl;

        this->odometry->Process(this->dataSource->Get());

        ++this->frameId;

        return true;
    }

private:
    void Loop()
    {
        if (this->isPlay)
        {
            Process();
            
            if (this->isStep)
            {
                Pause();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

protected:
    std::unique_ptr<IDataSource<T>> dataSource;
    std::unique_ptr<IOdometry<T>> odometry;
    std::unique_ptr<IRender> render;

    unsigned frameId;
    unsigned maxFrames;

    std::atomic<bool> isStarted;
    std::atomic<bool> isPlay;
    std::atomic<bool> isStep;

    std::function<void()> onProcessComplete;
};
    
} // namespace eacham
