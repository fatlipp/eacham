#pragma once

#include "data_source/IDataSource.h"
#include "odometry/IOdometry.h"
#include "visualization/IRender.h"
#include "config/Config.h"
#include "performance/BlockTimer.h"

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
        this->isStarted = true;

        while (this->isStarted)
        {
        }
    }

    void Stop()
    {
        this->isStarted = false;
    }

    void Play()
    {
        this->isPlay = true;

        while (this->isPlay && Process())
        {
            // loop
        }
    }

    void Pause()
    {
        this->isPlay = false;
    }

    void Step()
    {
        Pause();
        Process();
    }

protected:
    virtual bool Process()
    {
        if (this->frameId < this->maxFrames)
        {
            return false;
        }

        this->odometry->Process(this->dataSource->Get());
        this->render->Draw();

        ++this->frameId;

        return true;
    }

protected:
    std::unique_ptr<IDataSource<T>> dataSource;
    std::unique_ptr<IOdometry<T>> odometry;
    std::unique_ptr<IRender> render;

    unsigned frameId;
    unsigned maxFrames;

    std::atomic<bool> isStarted;
    std::atomic<bool> isPlay;

    std::function<void()> onProcessComplete;
};
    
} // namespace eacham
