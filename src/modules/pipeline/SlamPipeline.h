#pragma once

#include "pipeline/IPipeline.h"
#include "pipeline/OptimizerPipeline.h"
#include "data_source/IDataSource.h"
#include "frame/IFrameCreator.h"
#include "frame/FramePointData.h"
#include "odometry/IOdometry.h"
#include "map/Map.h"
#include "map/MapFrame.h"
#include "map/MapPoint.h"
#include "optimizer/IMapOptimizer.h"
#include "motion_estimator/IMotionEstimator.h"
#include "motion_estimator/EstimationResult.h"
#include "performance/BlockTimer.h"
#include "data_source/dataset/IDataset.h"

#include <future>
#include <atomic>
#include <thread>
#include <chrono>

namespace eacham
{

class SlamPipeline : public IPipeline
{
public:
    void SetMap(std::unique_ptr<Map> map)
    {
        this->map = std::move(map);
    }

    void SetDataSource(std::unique_ptr<IDataSourceCamera<stereodata_t>> dataSource)
    {
        this->dataSource = std::move(dataSource);
    }

    void SetFrameCreator(std::unique_ptr<IFrameCreator> frameCreator)
    {
        this->frameCreator = std::move(frameCreator);
    }

    void SetOdometry(std::unique_ptr<IOdometry> odometry)
    {
        this->odometry = std::move(odometry);
    }

    void SetOptimizer(std::unique_ptr<IMapOptimizer> optimizer)
    {
        optimizer->SetMap(this->map.get());

        this->optimizerPipeline = std::make_unique<OptimizerPipeline>(std::move(optimizer));
    }

    void SetOnUpdate(std::function<void(const Eigen::Matrix4f&)> callback)
    {
        this->callback = callback;
    }

    void Activate() override
    {
        this->map->Reset();

        IPipeline::Activate();

        this->optimizerPipeline->Activate();
    }

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
    void Process() override
    {
        if (!isPlay)
        {
            return;
        }

        {
            BlockTimer timer("SlamPipeline 1", true);

            const auto data = this->dataSource->Get();
            const auto frame = this->frameCreator->Create(data);

            if (!frame.isValid())
            {
                std::cout << "SlamPipeline() Invalid frame" << std::endl;

                Pause();
                return;
            }

            const EstimationResult estimation = this->odometry->Process(frame);

            if (estimation.isValid())
            {
                if (!this->map->AddFrame(frame.GetPointsData(), estimation))
                {
                    IPipeline::Deactivate();
                }

                optimizerPipeline->NeedOptimize();

                {
                    const auto lastFrame = this->map->GetFrames().back();

                    std::cout << "****************************************************id: " << lastFrame.id << std::endl;

                    auto currentPos = lastFrame.position;
                    auto dataset = dynamic_cast<IDataset*>(this->dataSource.get());
                    auto gtPos = std::get<1>(dataset->GetGtPose());

                    if (callback != nullptr)
                    {
                        callback.operator()(currentPos);
                    }

                    Eigen::Matrix4f diff = (currentPos - gtPos);
                    const float diffLen = std::sqrt(diff(0, 3) * diff(0, 3) + 
                                                    diff(1, 3) * diff(1, 3) + 
                                                    diff(2, 3) * diff(2, 3));

                    std::cout << "Current pos:\n" << currentPos << std::endl;
                    std::cout << "GT pos:\n" << gtPos << std::endl;
                    std::cout << "Diff:\n" << diffLen << "\n" << diff << std::endl;

                    if (lastFrame.id > 199)
                    {
                        std::cout << "SlamPipeline() Completed" << std::endl;

                        Pause();
                    }
                }
            }
            else
            {
                std::cout << "SlamPipeline() ESTIMATION ERROR" << std::endl;

                Pause();
            }
        }

        if (isStep)
        {
            Pause();
        }

        std::cout << "SlamPipeline() Process END\n\n" << std::endl;
    }

protected:
    std::unique_ptr<IDataSourceCamera<stereodata_t>> dataSource;
    std::unique_ptr<IFrameCreator> frameCreator;
    std::unique_ptr<IOdometry> odometry;
    std::unique_ptr<Map> map;
    std::unique_ptr<OptimizerPipeline> optimizerPipeline;

    std::function<void(const Eigen::Matrix4f&)> callback;

    std::atomic<bool> isPlay;
    std::atomic<bool> isStep;
    std::atomic<bool> isReset;

};

}