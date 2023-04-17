#pragma once

#include "data_source/IDataSource.h"
#include "odometry/IOdometry.h"
#include "odometry/IVisualOdometry.h"
#include "map/IMap.h"
#include "optimizer/IMapOptimizer.h"
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
        , isReset(false)
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

    void SetOdometry(std::unique_ptr<IVisualOdometry<T>> odometryInp)
    {
        this->odometry = std::move(odometryInp);
    }

    void SetMap(std::unique_ptr<IMap> mapInp)
    {
        this->map = std::move(mapInp);
    }

    void SetOptimizer(std::unique_ptr<IMapOptimizer> optimizerInp)
    {
        this->optimizer = std::move(optimizerInp);
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
        if (this->map != nullptr)
        {
            this->odometry->SetMap(this->map.get());
            this->optimizer->SetMap(this->map.get());
        }

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

    void Reset()
    {
        this->isReset = true;
    }

protected:
    virtual bool Process()
    {
        std::cout << "frameId: " << this->frameId << std::endl;

        bool isProcessed  = true;
        {
            BlockTimer timer("Pipeline::Process()");

            const auto data = this->dataSource->Get();

            static int lostCount = 0;

            isProcessed = this->odometry->Process(data);

            if (!isProcessed)
            {
                ++lostCount;

                if (lostCount > 10)
                {
                    this->isPlay = false;
                }
            }
            else
            {
                lostCount = 0;

                this->optimizer->Optimize();
            }
        }

        cv::waitKey(1);

        std::cout << "Current pos:\n" << this->odometry->GetPosition() << std::endl;

        ++this->frameId;

        return isProcessed;
    }

    virtual void CheckReset()
    {
        if (this->isReset)
        {
            Pause();

            this->odometry->Reset();

            this->frameId = 0;
            this->isReset = false;
        }
    }

private:
    void Loop()
    {
        while (this->isRunning)
        {
            if (this->maxFrames == 0 || this->frameId < this->maxFrames)
            {
                if (this->isPlay)
                {
                    std::cout << "[=============== PIPELINE PROCESS =============== ]" << std::endl;
                    Process();
                    std::cout << "[===============+================+=============== ]" << std::endl << std::endl;
                }
                
                if (this->isStep)
                {
                    Pause();
                }
            }
            
            CheckReset();
        }
    }

protected:
    std::unique_ptr<IDataSource<T>> dataSource;
    std::unique_ptr<IVisualOdometry<T>> odometry;
    std::unique_ptr<IMap> map;
    std::unique_ptr<IMapOptimizer> optimizer;
    std::unique_ptr<IRender> render;

    std::future<void> pipelineThread;
    std::atomic<bool> isRunning;

    unsigned frameId;
    unsigned maxFrames;

    std::atomic<bool> isPlay;
    std::atomic<bool> isStep;
    std::atomic<bool> isReset;

    std::function<void()> onProcessComplete;
};
    
} // namespace eacham
