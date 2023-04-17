#include <iostream>
#include "data_source/VisualDataSourceDirector.h"
#include "data_source/dataset/IDataset.h"
#include "odometry/VisualOdometryDirector.h"
#include "performance/BlockTimer.h"
#include "config/Config.h"
#include "pipeline/Pipeline.h"
#include "pipeline/PipelineDataset.h"
#include "map/MapFactory.h"
#include "optimizer/OptimizerFactory.h"
#include "visualization/Render.h"
#include "visualization/view/VisualOdometryF2FView.h"
#include "visualization/view/DatasetView.h"

using namespace eacham;

void MakeDataset()
{
}

int main(int argc, char* argv[])
{
    if (argc == 1)
    {
        std::cout << "Usage: `main 'config folder'`" << std::endl;
        return 1;
    }

    Config config { argv[1] };
    if (!config.Read())
    {
        return 2;
    }

    using T = stereodata_t;

    // specific for visual sensor
    VisualDataSourceDirector<T> dataSourceDirector;
    auto dataSource = dataSourceDirector.Build(config);

    if (dataSource == nullptr)
    {
        std::cerr << "dataSource is null" << std::endl;

        return 3;
    }

    VisualOdometryDirector<T> visualOdometryDirector;
    auto odometry = visualOdometryDirector.Build(dataSource.get(), config.GetOdometry());

    if (odometry == nullptr)
    {
        std::cerr << "odometry is null" << std::endl;

        return 3;
    }

    auto optimizer = OptimizerFactory::Build(config, dataSource->GetParameters());
    auto map = MapFactory::Build(config);

    // general implementation
    std::unique_ptr<Pipeline<T>> pipeline;
    std::unique_ptr<Render> render = std::make_unique<Render>();
    render->SetOnPlayClick([&pipeline]() { pipeline->Play(); });
    render->SetOnStepClick([&pipeline]() { pipeline->Step(); });
    render->SetOnResetClick([&pipeline]() { pipeline->Reset(); });
    render->SetOnCloseClick([&pipeline, &dataSource]() { pipeline->Kill(); dataSource->Stop(); });

    if (dynamic_cast<IDataset*>(dataSource.get()) != nullptr)
    {
        pipeline = std::make_unique<PipelineDataset<T>>(config.GetGeneral());
        
        render->Add(std::make_unique<DatasetView<T>>(dynamic_cast<IDataset*>(dataSource.get())));
    }
    else
    {
        pipeline = std::make_unique<Pipeline<T>>(config.GetGeneral());
    }

    auto vo = dynamic_cast<IFrameToFrameOdometry<T>*>(odometry.get()) ;
    if (vo != nullptr)
    {
        render->Add(std::make_unique<VisualOdometryF2FView<T>>(vo));
    }
    
    pipeline->SetDataSource(std::move(dataSource));
    pipeline->SetOdometry(std::move(odometry));
    pipeline->SetMap(std::move(map));
    pipeline->SetOptimizer(std::move(optimizer));
    pipeline->SetRender(std::move(render));
    pipeline->Start();

    while (pipeline->IsActive())
    {
    }

    return 0;
}